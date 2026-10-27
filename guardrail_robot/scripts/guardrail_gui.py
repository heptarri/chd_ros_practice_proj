#!/usr/bin/env python3
"""PyQt5 control panel for the guardrail_robot simulation.

The window shows the front camera feed on the left and, on the right, a mode
selector (line following vs. manual driving) plus an rqt_robot_steering-style
joystick made of two vertical sliders (``Linear x`` and ``Angular z``).

It also subscribes to the 3D LiDAR (``/scan/points``) and flags a broken
right-hand guardrail: a status line in the right panel and a marker drawn on
the camera image (see ``rail_damage.py``).

Modes
-----
* Manual: the two sliders publish ``geometry_msgs/msg/Twist`` on ``/cmd_vel``.
* Line following: the same detection / control code as ``line_follower.py``
  (imported from ``line_following.py``) drives the robot from the camera.

The ``/cmd_vel`` commands are published at a steady rate by a QTimer, and the
ROS executor is pumped from the Qt event loop, so no extra thread is needed.

Usage:
    ros2 run guardrail_robot guardrail_gui
    ros2 run guardrail_robot guardrail_gui --ros-args -p speed:=2.0
"""

import os
import signal
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from PyQt5 import QtCore, QtGui, QtWidgets

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rail_damage
from line_following import SignFollower, default_config, image_msg_to_bgr


class CameraView(QtWidgets.QLabel):
    """Label that displays the (BGR) camera frame, scaled to fit."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(640, 360)
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setStyleSheet('background-color: #101418; color: #8b96a5;')
        self.setText('等待摄像头画面…')
        self._image = None

    def set_image(self, qimage):
        self._image = qimage
        self.setText('')
        self._refresh()

    def _refresh(self):
        if self._image is None:
            return
        pixmap = QtGui.QPixmap.fromImage(self._image).scaled(
            self.size(), QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.SmoothTransformation)
        self.setPixmap(pixmap)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._refresh()


class SteeringJoystick(QtWidgets.QWidget):
    """Two vertical sliders, matching rqt_robot_steering's steering widget.

    ``Linear x`` goes from ``vx_min`` to ``vx_max`` (up = forward) and
    ``Angular z`` from ``vw_min`` to ``vw_max`` (up = left turn).
    """

    value_changed = QtCore.pyqtSignal(float, float)

    SLIDER_RANGE = 1000  # integer slider resolution

    def __init__(self, vx_min=-4.0, vx_max=12.0, vw_min=-3.0, vw_max=3.0,
                 parent=None):
        super().__init__(parent)
        self._limits = {
            'linear': (float(vx_min), float(vx_max)),
            'angular': (float(vw_min), float(vw_max)),
        }
        self._sliders = {}
        self._value_labels = {}
        self._build_ui()

    def _build_ui(self):
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(24)
        layout.addWidget(self._make_column('Linear x', 'linear', '前 +  /  后 −'))
        layout.addWidget(self._make_column('Angular z', 'angular', '左 +  /  右 −'))

    def _make_column(self, title, key, hint):
        lo, hi = self._limits[key]

        title_label = QtWidgets.QLabel(title)
        title_label.setAlignment(QtCore.Qt.AlignHCenter)
        font = title_label.font()
        font.setBold(True)
        title_label.setFont(font)

        value_label = QtWidgets.QLabel('0.00')
        value_label.setAlignment(QtCore.Qt.AlignHCenter)
        value_label.setStyleSheet('font-family: monospace; font-size: 15px;')

        slider = QtWidgets.QSlider(QtCore.Qt.Vertical)
        slider.setRange(0, self.SLIDER_RANGE)
        slider.setTickPosition(QtWidgets.QSlider.TicksBothSides)
        slider.setTickInterval(100)
        slider.setMinimumHeight(180)
        slider.valueChanged.connect(
            lambda value, k=key: self._on_slider(k))

        hint_label = QtWidgets.QLabel(f'{hint}\n{lo:+.0f} … {hi:+.0f}')
        hint_label.setAlignment(QtCore.Qt.AlignHCenter)
        hint_label.setStyleSheet('color: #8b96a5;')

        column = QtWidgets.QVBoxLayout()
        column.setSpacing(4)
        column.addWidget(title_label)
        column.addWidget(value_label)
        column.addWidget(slider, stretch=1)
        column.addWidget(hint_label)

        self._sliders[key] = slider
        self._value_labels[key] = value_label
        self.set_slider(key, 0.0)

        widget = QtWidgets.QWidget()
        widget.setLayout(column)
        return widget

    def _to_value(self, key, slider_value):
        lo, hi = self._limits[key]
        return lo + (slider_value / float(self.SLIDER_RANGE)) * (hi - lo)

    def _to_slider(self, key, value):
        lo, hi = self._limits[key]
        return int(round((value - lo) / (hi - lo) * self.SLIDER_RANGE))

    def set_slider(self, key, value):
        slider = self._sliders[key]
        slider.blockSignals(True)
        slider.setValue(self._to_slider(key, value))
        slider.blockSignals(False)
        self._value_labels[key].setText(f'{value:+.2f}')

    def _on_slider(self, key):
        value = self._to_value(key, self._sliders[key].value())
        self._value_labels[key].setText(f'{value:+.2f}')
        self.value_changed.emit(self.linear(), self.angular())

    def linear(self):
        return self._to_value('linear', self._sliders['linear'].value())

    def angular(self):
        return self._to_value('angular', self._sliders['angular'].value())

    def stop(self):
        self.set_slider('linear', 0.0)
        self.set_slider('angular', 0.0)


class GuardrailWindow(QtWidgets.QMainWindow):

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.cfg = self._read_params()
        self.cam = rail_damage.make_camera_projection(self.cfg)
        self.sf = SignFollower(self.cfg)
        self.damage = {'found': False, 'x': 0.0, 'length': 0.0,
                       'rail_y': -4.3, 'stamp': 0.0}
        self.damage_point = None
        self.bridge = CvBridge()
        self.twist = Twist()
        self.last_image_time = 0.0
        self.mode = 'manual'

        self._build_ui()
        self._build_ros()
        self._build_timers()

    # ------------------------------------------------------------------ ROS
    def _read_params(self):
        cfg = default_config()
        for name, value in cfg.items():
            self.node.declare_parameter(name, value)
            cfg[name] = self.node.get_parameter(name).value
        for name, value in (('vx_min', -4.0), ('vx_max', 12.0),
                            ('vw_min', -3.0), ('vw_max', 3.0),
                            ('image_width', 640), ('image_height', 480),
                            ('camera_x', 2.36), ('camera_y', 0.0),
                            ('camera_z', 0.025)):
            self.node.declare_parameter(name, value)
            cfg[name] = self.node.get_parameter(name).value

        det = rail_damage.default_detection_config()
        for name, value in det.items():
            self.node.declare_parameter(name, value)
            det[name] = self.node.get_parameter(name).value
        self.det_cfg = det
        return cfg

    def _build_ros(self):
        self.image_sub = self.node.create_subscription(
            Image, self.cfg['image_topic'], self._on_image,
            qos_profile_sensor_data)
        self.cmd_pub = self.node.create_publisher(
            Twist, self.cfg['cmd_topic'], 10)
        self.cloud_sub = self.node.create_subscription(
            PointCloud2, self.det_cfg['scan_topic'], self._on_cloud,
            qos_profile_sensor_data)

    def _build_timers(self):
        self.publish_timer = QtCore.QTimer(self)
        self.publish_timer.timeout.connect(self._on_publish)
        self.publish_timer.start(50)

        self.status_timer = QtCore.QTimer(self)
        self.status_timer.timeout.connect(self._on_status_tick)
        self.status_timer.start(200)

    # ------------------------------------------------------------------- UI
    def _build_ui(self):
        self.setWindowTitle('护栏巡检小车控制台 / Guardrail Robot Control')
        self.resize(1120, 720)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(12)

        # Left: camera feed.
        camera_box = QtWidgets.QGroupBox('摄像头画面 / Camera')
        camera_layout = QtWidgets.QVBoxLayout(camera_box)
        self.camera = CameraView()
        camera_layout.addWidget(self.camera)

        # Right: mode + joystick + status.
        mode_box = QtWidgets.QGroupBox('控制模式 / Mode')
        mode_layout = QtWidgets.QVBoxLayout(mode_box)
        self.rb_follow = QtWidgets.QRadioButton('寻迹 (line following)')
        self.rb_manual = QtWidgets.QRadioButton('手动控制 (manual)')
        self.rb_manual.setChecked(True)
        self.mode_group = QtWidgets.QButtonGroup(self)
        self.mode_group.addButton(self.rb_follow)
        self.mode_group.addButton(self.rb_manual)
        mode_layout.addWidget(self.rb_follow)
        mode_layout.addWidget(self.rb_manual)

        rail_box = QtWidgets.QGroupBox('护栏检测 / Guardrail')
        rail_layout = QtWidgets.QVBoxLayout(rail_box)
        self.rail_status = QtWidgets.QLabel('护栏检测中…')
        self.rail_status.setWordWrap(True)
        rail_layout.addWidget(self.rail_status)

        steer_box = QtWidgets.QGroupBox('手动摇杆 / Steering')
        steer_layout = QtWidgets.QVBoxLayout(steer_box)
        self.steering = SteeringJoystick(
            self.cfg['vx_min'], self.cfg['vx_max'],
            self.cfg['vw_min'], self.cfg['vw_max'])
        steer_layout.addWidget(self.steering)
        self.btn_stop = QtWidgets.QPushButton('急停 / Stop')
        self.btn_stop.setMinimumHeight(36)
        steer_layout.addWidget(self.btn_stop)

        status_box = QtWidgets.QGroupBox('状态 / Status')
        status_layout = QtWidgets.QVBoxLayout(status_box)
        self.follow_status = QtWidgets.QLabel('手动控制模式：用摇杆控制车速与转向。')
        self.follow_status.setWordWrap(True)
        status_layout.addWidget(self.follow_status)

        side = QtWidgets.QVBoxLayout()
        side.setSpacing(12)
        side.addWidget(mode_box)
        side.addWidget(rail_box)
        side.addWidget(steer_box, stretch=1)
        side.addWidget(status_box)

        root.addWidget(camera_box, stretch=1)
        root.addLayout(side)

        self.statusBar().showMessage('就绪')

        self.mode_group.buttonToggled.connect(self._on_mode_changed)
        self.steering.value_changed.connect(self._on_manual)
        self.btn_stop.clicked.connect(self._on_stop)

    # ------------------------------------------------------------- callbacks
    def _on_mode_changed(self, _button=None, _checked=None):
        if self.rb_manual.isChecked():
            self.mode = 'manual'
            self.steering.setEnabled(True)
            self.follow_status.setText('手动控制模式：用摇杆控制车速与转向。')
            self._on_manual(self.steering.linear(), self.steering.angular())
        else:
            self.mode = 'line_follow'
            self.steering.setEnabled(False)
            self.twist = Twist()
            self.sf.reset()
            self.follow_status.setText('寻迹模式：等待摄像头画面…')

    def _on_manual(self, linear, angular):
        if self.mode == 'manual':
            self.twist.linear.x = linear
            self.twist.angular.z = angular

    def _on_stop(self):
        self.steering.stop()
        self.twist = Twist()
        self.rb_manual.setChecked(True)  # also fires _on_mode_changed

    def _on_image(self, msg):
        self.last_image_time = time.monotonic()
        try:
            frame = image_msg_to_bgr(msg)
        except Exception as exc:  # cv_bridge raises CvBridgeError
            self.node.get_logger().error(f'cv_bridge 转换失败: {exc}')
            return

        if self.mode == 'line_follow':
            v, w, status, _ = self.sf.update(frame, self.cfg)
            self.twist.linear.x = v
            self.twist.angular.z = w
            self.follow_status.setText(status)

        self._draw_damage(frame)

        h, w = frame.shape[:2]
        image = QtGui.QImage(frame.data, w, h, frame.strides[0],
                             QtGui.QImage.Format_BGR888).copy()
        self.camera.set_image(image)

    def _on_cloud(self, msg):
        points = rail_damage.cloud_to_xyz(msg)
        result = rail_damage.detect_rail_gap(points, self.det_cfg)
        now = time.monotonic()

        if result is None:
            self.damage['stamp'] = now
            self.damage['found'] = False
            self.damage_point = None
            self._set_rail_status('unknown', 0.0)
            return

        self.damage['found'] = result['found']
        self.damage['x'] = result['x']
        self.damage['length'] = result['length']
        self.damage['rail_y'] = result['rail_y']
        self.damage['stamp'] = now

        if result['found']:
            # Mark the gap at the W-beam centre height in the base_link frame.
            self.damage_point = (result['x'], result['rail_y'], -0.175)
            self._set_rail_status('damage', result['x'])
        else:
            self.damage_point = None
            self._set_rail_status('ok', 0.0)

    def _set_rail_status(self, state, distance):
        if state == 'damage':
            where = '前方' if distance >= 0 else '后方'
            self.rail_status.setText(
                f'<span style="color:#e74c3c; font-weight:bold;">'
                f'⚠ 发现护栏破损（{where} {abs(distance):.1f} m）</span>')
        elif state == 'ok':
            self.rail_status.setText(
                '<span style="color:#2ecc71;">护栏连续 ✓</span>')
        else:
            self.rail_status.setText(
                '<span style="color:#8b96a5;">护栏检测中…</span>')

    def _draw_damage(self, frame):
        if not self.damage['found'] or self.damage_point is None:
            return
        if time.monotonic() - self.damage['stamp'] > 1.0:
            return
        h, w = frame.shape[:2]

        # Only mark it when the gap is still in front of the camera.
        if self.damage['x'] >= 2.36:
            pixel = rail_damage.project_to_pixel(self.damage_point, self.cam)
            if pixel is not None:
                u = int(round(pixel[0]))
                v = int(round(pixel[1]))
                if 0 <= u < w and 0 <= v < h:
                    cv2.rectangle(frame, (u - 24, v - 16), (u + 24, v + 16),
                                  (0, 0, 255), 2)
                    cv2.putText(frame, 'DAMAGE', (u - 22, v - 22),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255),
                                2, cv2.LINE_AA)
                    return

            # Detected but just outside the camera field of view: show a
            # marker pinned to the right edge (the guardrail is on the right).
            cv2.rectangle(frame, (w - 12, 0), (w, h), (0, 0, 255), -1)
            cv2.putText(frame, f'DAMAGE {self.damage["x"]:+.1f}m',
                        (w - 190, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 0, 255), 2, cv2.LINE_AA)

    def _on_publish(self):
        # Watchdog: stop if line following but the camera went silent.
        if self.mode == 'line_follow' and self.last_image_time > 0.0:
            if time.monotonic() - self.last_image_time > self.cfg['lost_timeout']:
                self.twist = Twist()
                self.follow_status.setText('摄像头无信号，已停车')
        self.cmd_pub.publish(self.twist)

    def _on_status_tick(self):
        mode = '寻迹' if self.mode == 'line_follow' else '手动'
        self.statusBar().showMessage(
            f"模式 {mode}  |  image: {self.cfg['image_topic']}  |  "
            f"cmd: {self.cfg['cmd_topic']}  |  "
            f"v={self.twist.linear.x:+.2f} m/s  "
            f"w={self.twist.angular.z:+.2f} rad/s")

    # ------------------------------------------------------------- shutdown
    def stop_robot(self):
        if not rclpy.ok():
            return
        self.twist = Twist()
        for _ in range(3):
            try:
                self.cmd_pub.publish(self.twist)
            except Exception:
                break
            time.sleep(0.02)

    def closeEvent(self, event):
        self.stop_robot()
        super().closeEvent(event)


def _pick_cjk_font():
    """Return the first installed font able to render simplified Chinese."""
    wanted = [
        'Noto Sans CJK SC', 'Noto Sans SC', 'Source Han Sans SC',
        'WenQuanYi Micro Hei', 'WenQuanYi Zen Hei', 'PingFang SC',
        'Microsoft YaHei', 'SimHei',
    ]
    families = set(QtGui.QFontDatabase().families())
    for name in wanted:
        if name in families:
            return name
    return None


def main(args=None):
    rclpy.init(args=args)
    node = Node('guardrail_gui')

    app = QtWidgets.QApplication(sys.argv)
    cjk_font = _pick_cjk_font()
    if cjk_font:
        app.setFont(QtGui.QFont(cjk_font, 10))

    window = GuardrailWindow(node)
    window.show()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_timer = QtCore.QTimer()
    spin_timer.timeout.connect(lambda: executor.spin_once(timeout_sec=0))
    spin_timer.start(20)

    # Ctrl+C / SIGTERM should close the Qt window cleanly.
    def _quit(*_args):
        QtWidgets.QApplication.instance().quit()

    signal.signal(signal.SIGINT, _quit)
    signal.signal(signal.SIGTERM, _quit)

    try:
        app.exec_()
    finally:
        spin_timer.stop()
        window.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
