#!/usr/bin/env python3
"""Pure line-following logic shared by the ROS node and the PyQt GUI.

Everything in this module is independent of rclpy and Qt, so it can be
imported both by ``line_follower.py`` (headless ROS node) and by
``guardrail_gui.py`` (PyQt5 control panel) without duplicating the detection
or the control law.
"""

import math
import time

import cv2
import numpy as np


def image_msg_to_bgr(msg):
    """Convert a sensor_msgs/Image to a BGR numpy array.

    Fast path for 8-bit ``rgb8`` / ``bgr8`` images (no cv_bridge, which is
    slow for large frames); falls back to cv_bridge for other encodings.
    """
    if msg.encoding in ('bgr8', 'rgb8') and msg.step >= msg.width * 3:
        data = np.frombuffer(msg.data, dtype=np.uint8)
        arr = data.reshape(msg.height, msg.step)[:, :msg.width * 3]
        arr = arr.reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            arr = np.ascontiguousarray(arr[:, :, ::-1])  # rgb -> bgr
        return arr
    from cv_bridge import CvBridge
    return CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')


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
        'speed': 3.0,
        'min_speed': 1.2,
        'error_slow': 3.0,
        # Topics / behaviour.
        'image_topic': '/camera/image_raw',
        'cmd_topic': '/cmd_vel',
        'lost_timeout': 0.6,
        # Road signs (coloured panels on the shoulder): yellow = stop for a
        # few seconds, blue = slow down then accelerate again.
        'sign_min_area': 800,
        'sign_stop_duration': 3.0,
        'sign_slow_duration': 3.0,
        'sign_slow_speed': 1.0,
        'sign_cooldown': 6.0,
        # Detection runs on a downscaled image (1.0 = full resolution) to
        # lighten the per-frame CPU load.
        'detect_scale': 0.5,
        'debug': False,
    }


def detect_and_steer(frame_bgr, cfg, hsv=None):
    """Detect the right edge line and compute a Twist command.

    ``hsv`` may be precomputed by the caller to avoid a redundant colour
    conversion when several detectors run on the same frame.

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

    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV) if hsv is None else hsv
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


def detect_sign(frame_bgr, cfg, hsv=None):
    """Detect a yellow or blue road sign in the camera image.

    Returns {'type': 'yellow'|'blue', 'area': n_pixels} or None.  Only highly
    saturated colours are matched, so the grey road, white lines and the
    pale-blue sky are ignored.
    """
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV) if hsv is None else hsv

    # Wide yellow range: the sign is amber/gold and lit/shaded differently in
    # the scene, so accept a broad hue/sat/value window.  Blue stays narrow.
    yellow = cv2.inRange(hsv, (15, 60, 60), (45, 255, 255))
    blue = cv2.inRange(hsv, (100, 120, 80), (130, 255, 255))

    yellow_area = int(cv2.countNonZero(yellow))
    blue_area = int(cv2.countNonZero(blue))
    threshold = cfg['sign_min_area']

    if yellow_area >= threshold and yellow_area >= blue_area:
        return {'type': 'yellow', 'area': yellow_area}
    if blue_area >= threshold and blue_area > yellow_area:
        return {'type': 'blue', 'area': blue_area}
    return None


class SignFollower:
    """Line following plus road-sign behaviour.

    Yellow sign -> stop for ``sign_stop_duration`` seconds.
    Blue sign   -> slow down for ``sign_slow_duration`` seconds, then resume.

    ``update()`` returns (v, w, status, result) where ``result`` is the raw
    ``detect_and_steer`` output (or None).
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self.state = 'follow'        # follow | stop | slow
        self.until = 0.0             # monotonic deadline for stop/slow
        self.cooldown_until = 0.0    # ignore signs until this time
        self.last_sign = None

    def update(self, frame, cfg):
        now = time.monotonic()

        # Run the detectors on a downscaled image and share a single HSV
        # conversion between them -- the detection math is scale-invariant, so
        # this is much cheaper without changing the control behaviour.
        scale = float(cfg.get('detect_scale', 1.0))
        if scale < 1.0:
            f = cv2.resize(
                frame,
                (int(frame.shape[1] * scale), int(frame.shape[0] * scale)),
                interpolation=cv2.INTER_AREA)
            area = scale * scale
            local_cfg = dict(cfg)
            local_cfg['min_pixels'] = max(20, int(cfg['min_pixels'] * area))
            local_cfg['sign_min_area'] = max(50, int(cfg['sign_min_area'] * area))
        else:
            f = frame
            local_cfg = cfg

        hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)

        sign = None
        if now >= self.cooldown_until:
            sign = detect_sign(f, local_cfg, hsv)

        # A stop / slow behaviour is still running.
        if self.state in ('stop', 'slow') and now < self.until:
            if self.state == 'stop':
                return (0.0, 0.0, '黄色路标：停止中…', None)
            res = detect_and_steer(f, local_cfg, hsv)
            if res is None:
                return (0.0, 0.0, '蓝色路标：减速中（线丢失）…', None)
            return (local_cfg['sign_slow_speed'], res['w'],
                    '蓝色路标：减速中…', res)

        if self.state != 'follow':
            self.state = 'follow'  # timer expired, back to normal

        res = detect_and_steer(f, local_cfg, hsv)

        if sign is not None:
            if sign['type'] == 'yellow':
                self.state = 'stop'
                self.until = now + cfg['sign_stop_duration']
                self.cooldown_until = (now + cfg['sign_stop_duration']
                                       + cfg['sign_cooldown'])
                self.last_sign = 'yellow'
                return (0.0, 0.0, '检测到黄色路标：停止 3 秒…', res)
            self.state = 'slow'
            self.until = now + cfg['sign_slow_duration']
            self.cooldown_until = (now + cfg['sign_slow_duration']
                                   + cfg['sign_cooldown'])
            self.last_sign = 'blue'
            w = res['w'] if res is not None else 0.0
            return (cfg['sign_slow_speed'], w, '检测到蓝色路标：减速…', res)

        if res is None:
            return (0.0, 0.0, '丢失车道线，已停车', None)
        return (res['v'], res['w'],
                f"寻迹 v={res['v']:.1f} 偏移 {res['lateral']:+.2f}", res)
