#!/usr/bin/env python3
"""Serve the React web UI and bridge it to the guardrail robot.

This replaces the PyQt control panel with a browser UI.  All the ROS work
(camera, line following, guardrail damage detection, /cmd_vel publishing)
stays in this node; the browser only displays data and sends small commands.

Transport (stdlib only, no extra Python packages):
  * GET  /         -> the React app (web/index.html, /vendor, /app.jsx, css)
  * GET  /stream   -> Server-Sent Events: camera JPEG frames and telemetry
  * POST /cmd      -> JSON commands: {"mode": "manual"|"follow",
                      "linear": .., "angular": .., "stop": true}

Usage:
    ros2 run guardrail_robot webui_server.py
    ros2 launch guardrail_robot webui.launch.py
Then open http://localhost:8000 in a browser.
"""

import base64
import json
import os
import queue
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rail_damage
from line_following import SignFollower, default_config, image_msg_to_bgr


class WebUiServer(Node):

    def __init__(self):
        super().__init__('webui_server')

        self.declare_parameter('port', 8000)
        self.port = int(self.get_parameter('port').value)

        self.cfg = default_config()
        for name, value in self.cfg.items():
            self.declare_parameter(name, value)
            self.cfg[name] = self.get_parameter(name).value
        for name, value in (('image_width', 640), ('image_height', 480),
                            ('camera_x', 2.36), ('camera_y', 0.0),
                            ('camera_z', 0.025)):
            self.declare_parameter(name, value)
            self.cfg[name] = self.get_parameter(name).value

        self.det_cfg = rail_damage.default_detection_config()
        for name, value in self.det_cfg.items():
            self.declare_parameter(name, value)
            self.det_cfg[name] = self.get_parameter(name).value

        self.cam = rail_damage.make_camera_projection(self.cfg)
        self.sf = SignFollower(self.cfg)
        self.bridge = CvBridge()

        self.mode = 'manual'
        self.twist = Twist()
        self.follow_status = '手动控制模式'
        self.last_image_time = 0.0
        self.damage = {'found': False, 'x': 0.0, 'length': 0.0,
                       'rail_y': -4.3, 'stamp': 0.0}
        self.damage_point = None
        self.damage_known = False
        self.latest_jpeg = None

        self.state_lock = threading.Lock()
        self.sse_clients = set()
        self.sse_lock = threading.Lock()

        # ROS: camera + point cloud in, /cmd_vel out.
        self.image_sub = self.create_subscription(
            Image, self.cfg['image_topic'], self.on_image,
            qos_profile_sensor_data)
        self.cloud_sub = self.create_subscription(
            PointCloud2, self.det_cfg['scan_topic'], self.on_cloud,
            qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(
            Twist, self.cfg['cmd_topic'], 10)

        self.create_timer(0.05, self.publish_cmd)
        self.create_timer(0.2, self.broadcast_state)

        self.web_dir = os.path.join(
            get_package_share_directory('guardrail_robot'), 'web')

        self.get_logger().info(
            f"webui_server serving {self.web_dir} on port {self.port}")

    # --------------------------------------------------------------- ROS
    def on_image(self, msg):
        self.last_image_time = time.monotonic()
        try:
            frame = image_msg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().error(f'cv_bridge failed: {exc}')
            return

        with self.state_lock:
            if self.mode == 'follow':
                v, w, status, _ = self.sf.update(frame, self.cfg)
                self.twist.linear.x = v
                self.twist.angular.z = w
                self.follow_status = status

        self._draw_damage(frame)
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if not ok:
            return
        jpeg = base64.b64encode(buf.tobytes()).decode('ascii')
        with self.state_lock:
            self.latest_jpeg = jpeg
        self.broadcast(json.dumps({'type': 'image', 'jpeg': jpeg}))

    def on_cloud(self, msg):
        points = rail_damage.cloud_to_xyz(msg)
        result = rail_damage.detect_rail_gap(points, self.det_cfg)
        now = time.monotonic()
        with self.state_lock:
            if result is None:
                self.damage['stamp'] = now
                self.damage['found'] = False
                self.damage_point = None
                return
            self.damage_known = True
            self.damage['found'] = result['found']
            self.damage['x'] = result['x']
            self.damage['length'] = result['length']
            self.damage['rail_y'] = result['rail_y']
            self.damage['stamp'] = now
            if result['found']:
                self.damage_point = (result['x'], result['rail_y'], -0.175)
            else:
                self.damage_point = None

    def publish_cmd(self):
        with self.state_lock:
            if (self.mode == 'follow' and self.last_image_time > 0.0 and
                    time.monotonic() - self.last_image_time >
                    self.cfg['lost_timeout']):
                self.twist = Twist()
            twist = self.twist
        self.cmd_pub.publish(twist)

    # ------------------------------------------------------------- control
    def apply_command(self, cmd):
        with self.state_lock:
            if 'mode' in cmd and cmd['mode'] in ('manual', 'follow'):
                self.mode = cmd['mode']
                self.twist = Twist()
                self.sf.reset()
                self.follow_status = ('手动控制模式' if self.mode == 'manual'
                                      else '等待摄像头…')
            if cmd.get('stop'):
                self.twist = Twist()
                self.mode = 'manual'
                self.follow_status = '手动控制模式'
            if (self.mode == 'manual' and
                    'linear' in cmd and 'angular' in cmd):
                self.twist.linear.x = float(cmd['linear'])
                self.twist.angular.z = float(cmd['angular'])
            return {'ok': True, 'mode': self.mode}

    # ---------------------------------------------------------- damage
    def _draw_damage(self, frame):
        with self.state_lock:
            found = self.damage['found']
            point = self.damage_point
            x = self.damage['x']
            fresh = (time.monotonic() - self.damage['stamp']) < 1.0
        if not found or point is None or not fresh:
            return
        h, w = frame.shape[:2]
        if x >= 2.36:
            pixel = rail_damage.project_to_pixel(point, self.cam)
            if pixel is not None:
                u, v = int(round(pixel[0])), int(round(pixel[1]))
                if 0 <= u < w and 0 <= v < h:
                    cv2.rectangle(frame, (u - 24, v - 16), (u + 24, v + 16),
                                  (0, 0, 255), 2)
                    cv2.putText(frame, 'DAMAGE', (u - 22, v - 22),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255),
                                2, cv2.LINE_AA)
                    return
            cv2.rectangle(frame, (w - 12, 0), (w, h), (0, 0, 255), -1)
            cv2.putText(frame, f'DAMAGE {x:+.1f}m', (w - 190, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2,
                        cv2.LINE_AA)

    # ------------------------------------------------------------ SSE
    def broadcast(self, message):
        with self.sse_lock:
            clients = list(self.sse_clients)
        for q in clients:
            try:
                q.put_nowait(message)
            except queue.Full:
                pass

    def broadcast_state(self):
        with self.state_lock:
            fresh = (time.monotonic() - self.damage['stamp']) < 1.0
            if self.damage_known:
                dstate = 'damage' if (self.damage['found'] and fresh) else 'ok'
            else:
                dstate = 'unknown'
            state = {
                'type': 'state',
                'mode': self.mode,
                'v': self.twist.linear.x,
                'w': self.twist.angular.z,
                'follow': self.follow_status,
                'damage': {
                    'state': dstate,
                    'x': self.damage['x'],
                    'length': self.damage['length'],
                },
            }
        self.broadcast(json.dumps(state))


# ------------------------------------------------------------- HTTP handler
def make_handler(node):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass  # keep the console quiet

        def _resolve(self):
            rel = urlparse(self.path).path.lstrip('/')
            if rel == '':
                rel = 'index.html'
            root = os.path.realpath(node.web_dir)
            full = os.path.realpath(os.path.join(root, rel))
            if not full.startswith(root + os.sep) and full != root:
                return None
            return full

        def _serve_file(self, path):
            if path is None or not os.path.isfile(path):
                self.send_error(404, 'not found')
                return
            ext = os.path.splitext(path)[1].lower()
            ctype = {
                '.html': 'text/html; charset=utf-8',
                '.js': 'text/javascript; charset=utf-8',
                '.jsx': 'text/javascript; charset=utf-8',
                '.css': 'text/css; charset=utf-8',
                '.svg': 'image/svg+xml',
            }.get(ext, 'application/octet-stream')
            data = open(path, 'rb').read()
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _stream(self):
            q = queue.Queue(maxsize=8)
            with node.sse_lock:
                node.sse_clients.add(q)
            try:
                self.send_response(200)
                self.send_header('Content-Type', 'text/event-stream')
                self.send_header('Cache-Control', 'no-cache')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(b': connected\n\n')
                self.wfile.flush()
                while True:
                    try:
                        item = q.get(timeout=15)
                    except queue.Empty:
                        self.wfile.write(b': ping\n\n')
                        self.wfile.flush()
                        continue
                    self.wfile.write(f'data: {item}\n\n'.encode('utf-8'))
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                with node.sse_lock:
                    node.sse_clients.discard(q)

        def do_GET(self):
            if urlparse(self.path).path == '/stream':
                self._stream()
                return
            self._serve_file(self._resolve())

        def do_POST(self):
            if urlparse(self.path).path != '/cmd':
                self.send_error(404, 'not found')
                return
            length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(length).decode('utf-8', 'replace')
            try:
                cmd = json.loads(body)
            except json.JSONDecodeError:
                self._send_json(400, {'error': 'bad json'})
                return
            result = node.apply_command(cmd)
            self._send_json(200, result)

        def _send_json(self, status, obj):
            data = json.dumps(obj).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(data)

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = WebUiServer()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    handler = make_handler(node)
    server = ThreadingHTTPServer(('0.0.0.0', node.port), handler)
    node.get_logger().info(f'web UI at http://localhost:{node.port}')

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
