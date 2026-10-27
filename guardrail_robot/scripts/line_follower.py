#!/usr/bin/env python3
"""Camera-based road-line follower for the guardrail_robot simulation.

The node subscribes to the front camera, detects the solid white right-hand
edge line and publishes geometry_msgs/Twist on /cmd_vel.  The car is kept at a
fixed lateral distance from that line (default: 1.775 m, i.e. the centre of
the right lane), so it follows the lane through the bend.

The detection / control code lives in ``line_following.py`` so that both this
headless node and the PyQt GUI (``guardrail_gui.py``) share the same logic.

Usage:
    ros2 run guardrail_robot line_follower.py
    ros2 run guardrail_robot line_follower.py --ros-args -p speed:=2.0
    ros2 run guardrail_robot line_follower.py --ros-args -p debug:=true

The optional debug image is published on /line_follower/debug and can be
viewed with rqt_image_view.
"""

import os
import signal
import sys
import time

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from line_following import SignFollower, default_config, detect_and_steer


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
        self.last_status_log = 0.0
        self.sf = SignFollower(self.cfg)

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
        v, w, status, result = self.sf.update(frame, self.cfg)
        self.cmd.linear.x = v
        self.cmd.angular.z = w

        now = time.monotonic()
        if now - self.last_status_log > 1.0:
            self.get_logger().info(f'sign follower: {status}')
            self.last_status_log = now

        if (self.debug_pub is not None and result is not None
                and result['overlay'] is not None):
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
