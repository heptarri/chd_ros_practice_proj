#!/usr/bin/env python3
"""Camera-based road-line follower for the guardrail_robot simulation.

The node subscribes to the front camera, detects the solid white right-hand
edge line and publishes geometry_msgs/Twist on /cmd_vel.  The car is kept at a
fixed lateral distance from that line (default: 1.775 m, i.e. the centre of
the right lane), so it follows the lane through the bend.

Usage:
    ros2 run guardrail_robot line_follower.py
    ros2 run guardrail_robot line_follower.py --ros-args -p speed:=2.0
    ros2 run guardrail_robot line_follower.py --ros-args -p debug:=true

The optional debug image is published on /line_follower/debug and can be
viewed with rqt_image_view.
"""

import math
import signal
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def default_config():
    """Default parameters, also used as ROS parameter defaults."""
    return {
        # Camera model (must match urdf/sensors.xacro).
        'hfov': 1.396263,
        'camera_height': 0.80,
        'camera_yaw': -0.08,
        'camera_pitch': 0.0,
        # Detection region: right part of the image, below the horizon.
        'roi_x_min': 0.45,
        'roi_y_min': 0.60,
        'roi_y_max': 0.95,
        'white_v_min': 150,
        'white_s_max': 70,
        'min_pixels': 120,
        # Control.
        'look_ratio': 0.72,
        'target_offset': -1.775,
        'kp': 0.9,
        'max_angular': 1.0,
        'speed': 7,
        'min_speed': 0.5,
        'error_slow': 1.5,
        # Topics / behaviour.
        'image_topic': '/camera/image_raw',
        'cmd_topic': '/cmd_vel',
        'lost_timeout': 0.6,
        'debug': False,
    }


def detect_and_steer(frame_bgr, cfg):
    """Detect the right edge line and compute a Twist command.

    Returns a dict with v, w, error, lateral, x, row, slope, mask and an
    optional debug overlay, or None when no trustworthy line was found.
    """
    h, w = frame_bgr.shape[:2]
    if h < 32 or w < 32:
        return None

    focal = (w * 0.5) / math.tan(cfg['hfov'] * 0.5)
    cx, cy = w * 0.5, h * 0.5

    x0 = int(w * cfg['roi_x_min'])
    x1 = w
    y0 = int(h * cfg['roi_y_min'])
    y1 = int(h * cfg['roi_y_max'])
    y0 = max(0, min(h - 2, y0))
    y1 = max(y0 + 1, min(h, y1))

    roi = np.zeros((h, w), dtype=np.uint8)
    roi[y0:y1, x0:x1] = 255

    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(
        hsv,
        (0, 0, int(cfg['white_v_min'])),
        (180, int(cfg['white_s_max']), 255),
    )
    mask = cv2.bitwise_and(white, roi)

    if int(cv2.countNonZero(mask)) < cfg['min_pixels']:
        return None

    ys, xs = np.nonzero(mask)
    if ys.size < cfg['min_pixels']:
        return None

    # x as a function of y (the right edge line is strongly slanted).
    try:
        slope, intercept = np.polyfit(ys, xs, 1)
    except (np.linalg.LinAlgError, ValueError):
        return None

    # Drop stray bright pixels and fit once more.
    residual = np.abs(xs - (slope * ys + intercept))
    keep = residual <= max(12.0, 0.03 * w)
    if int(keep.sum()) >= cfg['min_pixels']:
        try:
            slope, intercept = np.polyfit(ys[keep], xs[keep], 1)
        except (np.linalg.LinAlgError, ValueError):
            return None

    if not (0.05 <= slope <= 15.0):
        return None

    look_row = int(round(h * cfg['look_ratio']))
    look_row = max(y0, min(y1 - 1, look_row))
    x_look = slope * look_row + intercept
    if not (0.0 <= x_look < w):
        return None

    # Ground distance of the look-ahead row (flat road, level/pitched camera).
    beta = math.atan2(look_row - cy, focal)
    theta = beta + cfg['camera_pitch']
    if theta <= 0.03:
        return None
    distance = cfg['camera_height'] / math.tan(theta)

    # Lateral offset of the line in the car frame (+left, -right).
    alpha = math.atan2(x_look - cx, focal)      # +right of the camera axis
    line_angle = cfg['camera_yaw'] - alpha      # angle in the car frame
    lateral = distance * math.tan(line_angle)

    error = lateral - cfg['target_offset']
    angular = cfg['kp'] * error
    angular = max(-cfg['max_angular'], min(cfg['max_angular'], angular))

    speed = cfg['speed'] - (cfg['speed'] - cfg['min_speed']) * min(
        abs(error) / cfg['error_slow'], 1.0)
    speed = max(cfg['min_speed'], speed)

    overlay = None
    if cfg['debug']:
        overlay = frame_bgr.copy()
        cv2.rectangle(overlay, (x0, y0), (x1 - 1, y1 - 1), (0, 255, 0), 1)
        overlay[mask > 0] = (0, 255, 0)
        xa = int(slope * y0 + intercept)
        xb = int(slope * (y1 - 1) + intercept)
        cv2.line(overlay, (xa, y0), (xb, y1 - 1), (255, 0, 0), 2)
        cv2.line(overlay, (0, look_row), (w - 1, look_row), (0, 0, 255), 1)
        cv2.circle(overlay, (int(x_look), look_row), 5, (0, 0, 255), -1)
        text = (f'lateral={lateral:+.2f}m err={error:+.2f}m '
                f'v={speed:.1f} w={angular:+.2f}')
        cv2.putText(overlay, text, (10, 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.60, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(overlay, text, (10, 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.60, (0, 0, 0), 1, cv2.LINE_AA)

    return {
        'v': float(speed),
        'w': float(angular),
        'error': float(error),
        'lateral': float(lateral),
        'x': float(x_look),
        'row': int(look_row),
        'slope': float(slope),
        'mask': mask,
        'overlay': overlay,
    }


class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')

        self.cfg = default_config()
        for name, value in self.cfg.items():
            self.declare_parameter(name, value)
            self.cfg[name] = self.get_parameter(name).value

        self.bridge = CvBridge()
        self.cmd = Twist()
        self.last_image_time = 0.0
        self.last_lost_log = 0.0

        self.create_subscription(
            Image, self.cfg['image_topic'], self.on_image,
            qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(
            Twist, self.cfg['cmd_topic'], 10)
        self.debug_pub = None
        if self.cfg['debug']:
            self.debug_pub = self.create_publisher(
                Image, '/line_follower/debug', 10)

        # Publish at a steady rate and hold the last command between images.
        self.create_timer(0.05, self.on_timer)

        self.get_logger().info(
            f"line follower: image={self.cfg['image_topic']} "
            f"cmd={self.cfg['cmd_topic']} speed={self.cfg['speed']:.1f} m/s "
            f"target_offset={self.cfg['target_offset']:+.2f} m")

    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # cv_bridge raises CvBridgeError
            self.get_logger().error(f'cv_bridge conversion failed: {exc}')
            return

        self.last_image_time = time.monotonic()
        result = detect_and_steer(frame, self.cfg)

        if result is None:
            # Losing the solid edge line means we should not keep driving.
            self.cmd = Twist()
            now = time.monotonic()
            if now - self.last_lost_log > 1.0:
                self.get_logger().warn('road line lost, stopping')
                self.last_lost_log = now
            return

        self.cmd.linear.x = result['v']
        self.cmd.angular.z = result['w']

        if self.debug_pub is not None and result['overlay'] is not None:
            debug = self.bridge.cv2_to_imgmsg(result['overlay'], encoding='bgr8')
            debug.header = msg.header
            self.debug_pub.publish(debug)

    def on_timer(self):
        # Watchdog: stop if the camera stops delivering images.
        if (self.last_image_time > 0.0 and
                time.monotonic() - self.last_image_time > self.cfg['lost_timeout']):
            self.cmd = Twist()
        self.cmd_pub.publish(self.cmd)

    def stop_robot(self):
        # Do not try to publish after the context has been shut down.
        if not rclpy.ok():
            return
        self.cmd = Twist()
        for _ in range(3):
            try:
                self.cmd_pub.publish(self.cmd)
            except Exception:
                break
            time.sleep(0.02)


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)
    # Let Ctrl+C / SIGTERM reach the finally block so a zero Twist is sent
    # before the process exits.  (rclpy's default handler shuts the context
    # down first, after which publishing is no longer possible.)
    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)

    node = LineFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_robot()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
