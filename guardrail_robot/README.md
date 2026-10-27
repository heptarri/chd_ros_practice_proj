# guardrail_robot

ROS 2 Humble + Gazebo（Fortress / ignition-gazebo6）中的护栏巡检小车仿真。

## 尺寸

小车按真实中型乘用车/高速公路巡检车尺寸建模，公路按实际两车道高速断面建模，
并做成半径 120 m、约 86° 的左弯：

| 项目 | 数值 |
| --- | --- |
| 车体（长 × 宽 × 高） | 4.70 m × 1.85 m × 1.50 m |
| 总高（含车顶设备/激光雷达） | 约 1.74 m |
| 轴距 / 轮距 | 2.80 m / 1.58 m |
| 车轮直径 / 宽度 | 0.68 m / 0.24 m |
| 离地间隙（底板） | 0.28 m |
| 整备质量 | 约 1600 kg |
| 相机 / 激光雷达离地高度 | 0.80 m / 1.70 m |
| 车道宽 | 3.75 m |
| 右侧硬路肩 | 2.50 m |
| 公路长度 / 弯道半径 / 转角 | 180 m / 120 m / 约 86°（左弯） |
| 中央虚线 | 6 m 线段 + 9 m 间隔 |
| 护栏梁高 / 立柱间距 | 0.31 m / 4 m |

## 修改赛道

`worlds/highway.sdf` 由 `tools/generate_highway.py` 生成。要改变长度或弯曲程度，
编辑脚本顶部的 `R`（弯道半径）、`S_START` / `S_END`（起止弧长），然后重新生成并编译：

```bash
python3 src/guardrail_robot/tools/generate_highway.py
colcon build --packages-select guardrail_robot
source install/setup.bash
```

当前配置为半径 120 m、总长 180 m（约 86° 左弯）。地图上不再放置交通锥桶/施工桶。

## 编译

```bash
cd ~/ros_prac
colcon build --packages-select guardrail_robot
source install/setup.bash
```

依赖（ROS 2 Humble）：`ros-humble-ros-gz-sim`、`ros-humble-ros-gz-bridge`、
`ros-humble-rqt-robot-steering`。缺少遥控插件时安装：

```bash
sudo apt install ros-humble-rqt-robot-steering
```

## 启动仿真

```bash
ros2 launch guardrail_robot sim.launch.py
```

默认会同时启动 Gazebo、`robot_state_publisher`、`ros_gz_bridge`、RViz 和
`rqt_robot_steering`。可用参数关闭不需要的图形界面：

```bash
ros2 launch guardrail_robot sim.launch.py rviz:=false teleop:=false
```

## PyQt 控制台（摄像头 + 模式 + 摇杆）

`scripts/guardrail_gui.py` 是一个 PyQt5 控制面板，左边显示摄像头实时画面，右边
提供：

* **控制模式**：`寻迹 (line following)` / `手动控制 (manual)` 单选切换；
* **手动摇杆**：与 `rqt_robot_steering` 同款的 `Linear x` / `Angular z` 两个
  竖直滑块（范围 −4…12 m/s、−3…3 rad/s），外加一个 **急停 / Stop** 按钮。

寻迹模式复用 `line_follower.py` 同一套检测/控制逻辑（抽到
`scripts/line_following.py`），因此面板可以单独完成寻迹；手动模式则由摇杆直接
往 `/cmd_vel` 发布 `Twist`。

启动时先关掉默认的 rqt 遥控，避免两个 `/cmd_vel` 发布者冲突：

```bash
ros2 launch guardrail_robot sim.launch.py teleop:=false
ros2 launch guardrail_robot gui.launch.py
```

也可以直接运行，或用 ROS 参数覆盖寻迹速度：

```bash
ros2 run guardrail_robot guardrail_gui
ros2 run guardrail_robot guardrail_gui --ros-args -p speed:=2.0
```

界面中文需要系统装有中文字体（程序会自动选用 Noto CJK / 文泉驿等）。若显示为
方框，安装中文字体即可：

```bash
sudo apt install fonts-noto-cjk        # 或 fonts-wqy-microhei
```

## Web UI（React）

`scripts/webui_server.py` 把上面的 PyQt 控制台移植成了浏览器页面：后端仍是 ROS 节点
（摄像头、寻迹、护栏破损检测、`/cmd_vel` 发布都在后端），前端用 React 编写，通过
SSE 接收画面与状态、通过 HTTP 发送控制命令。静态页面在 `web/`（React 18 + Babel
内联编译，无需 Node/npm 构建）。

```bash
# 先启动仿真（关掉默认 rqt 遥控）
ros2 launch guardrail_robot sim.launch.py teleop:=false
# 再启动 web 服务
ros2 launch guardrail_robot webui.launch.py
```

浏览器打开 <http://localhost:8000> 即可看到摄像头画面、控制模式切换、手动摇杆
（Linear x / Angular z）、急停、护栏破损提示。

- 端口默认 8000，可改：`ros2 launch guardrail_robot webui.launch.py port:=9000`。

## 路标识别（黄 / 蓝）

地图右侧路肩上有两块彩色路标（见 `tools/generate_highway.py` 里的 `SIGNS`）：

* **蓝色路标**（约 70 m）：代表需要减速，减速 3 秒后重新加速；
* **黄色路标**（约 110 m）：代表停车，停止 3 秒后继续。

识别逻辑在 `scripts/line_following.py` 的 `detect_sign()`（HSV 颜色分割，只匹配高饱和
的黄/蓝，忽略灰路面、白线和淡蓝天）和 `SignFollower`（状态机：停车/减速 + 冷却，避免
同一块路标反复触发）。寻迹节点、PyQt 控制台、Web UI 的寻迹模式都复用这套逻辑。

相关参数可用 `--ros-args -p ...` 调整：`sign_min_area`（触发面积）、
`sign_stop_duration`（黄标停车时长，默认 3 s）、`sign_slow_duration` /
`sign_slow_speed`（蓝标减速时长/速度，默认 3 s / 1 m/s）、`sign_cooldown`（冷却）。

## 护栏破损检测（激光雷达）

`scripts/rail_damage.py` 用三维激光雷达点云（`/scan/points`）识别右侧护栏的破损
（不连续）。原理：取护栏梁高度带的点、只保留车辆右侧、用这些点的中位横向位置
估计护栏位置，再沿车辆前进方向做直方图，出现一段两侧都有护栏、长度超过阈值
（默认 2.5 m）的空档即判定为破损。

GUI 会：

* 在右侧 **护栏检测 / Guardrail** 栏提示：`护栏连续 ✓`（绿）或
  `⚠ 发现护栏破损（前方 X m）`（红）；
* 在摄像头画面上把破损位置用红框标出；当破损暂时超出相机视野时，在画面右缘
  显示红色提示条。

检测参数（`z_min/z_max`、`min_gap`、`bin_size` 等）都可通过 ROS 参数覆盖，例如：

```bash
ros2 run guardrail_robot guardrail_gui --ros-args -p min_gap:=3.0
```

## 用 rqt_robot_steering 遥控

`sim.launch.py` 已经启动了一个配置好的 `rqt_robot_steering` 窗口，直接拖动
**Linear x**（前进/后退）和 **Angular z**（左转/右转）滑块即可。前进速度
范围 0–12 m/s（43 km/h），倒车 0–4 m/s，转向 ±3 rad/s。松开滑块前小车会
持续按当前速度行驶，把滑块拖回 0 或点击 **Stop** 停车。

也可以单独启动遥控窗口（仿真已经在运行时）：

```bash
ros2 launch guardrail_robot teleop.launch.py
```

### 为什么需要单独的 teleop launch 文件

Gazebo 的 DiffDrive 插件订阅 `/cmd_vel`，消息类型是
`geometry_msgs/msg/Twist`。而 `rqt_robot_steering` 会把上一次使用的
“话题名”和 “stamped” 复选框保存到 `~/.config/ros.org/rqt_gui.ini`，所以：

* 话题可能还停留在别的工程（例如 `.../cmd_vel_unstamped`）；
* 勾选 `stamped` 后会发布 `geometry_msgs/msg/TwistStamped`，类型不匹配，
  `ros_gz_bridge` 不会转发，小车不动。

`teleop.launch.py` 会为 rqt 使用一份独立的、已预设为
`topic=/cmd_vel`、`stamped=false` 的配置（`config/rqt_robot_steering.ini`），
因此不会受全局 rqt 配置影响。

同时它使用 `scripts/rqt_robot_steering_guardrail` 启动 rqt：
rqt_robot_steering 1.0.3 在 PyQt5 下把浮点数传给 `QSlider.setMaximum()`，
会抛 `TypeError`，导致速度上限设置失效、滑块始终只有 ±1 m/s。包装脚本把
该值转成整数，滑块才能使用 0–12 m/s 的速度范围。

如果不想通过 launch 文件启动，也可以直接运行包装脚本：

```bash
ros2 run guardrail_robot rqt_robot_steering_guardrail
```

注意：直接运行系统自带的 `ros2 run rqt_robot_steering rqt_robot_steering`
时，受上述 bug 影响滑块最大仍只有 ±1 m/s，车子会显得很慢。

然后确认窗口中的 **Topic** 为 `/cmd_vel`，并**取消勾选 stamped**。

### 速度参数

* Gazebo 侧上限：`urdf/gazebo.xacro` 中的 `max_linear_velocity`
  （当前 12 m/s）、`max_linear_acceleration`（8 m/s²）。
* rqt 滑块范围：`config/rqt_robot_steering.ini` 中的 `vx_max` / `vx_min`。

## 话题说明

| 方向 | ROS 话题 | 类型 |
| --- | --- | --- |
| 控制输入 | `/cmd_vel` | `geometry_msgs/msg/Twist` |
| 里程计 | `/odom` | `nav_msgs/msg/Odometry` |
| 激光雷达（单线扫描） | `/scan` | `sensor_msgs/msg/LaserScan` |
| 激光雷达（三维点云） | `/scan/points` | `sensor_msgs/msg/PointCloud2` |
| 相机 | `/camera/image_raw` | `sensor_msgs/msg/Image` |
| IMU | `/imu` | `sensor_msgs/msg/Imu` |

## 常见问题

### RViz 反复提示 `Detected jump back in time`，点云/模型不显示

这是**时间戳冲突**，不是点云话题的问题。通常是因为同时（或之前残留）运行了
多个 Gazebo / bridge / robot_state_publisher 进程：多个 `/clock`、
`/joint_states`、`/tf` 发布者带着不同的仿真时间交错到达，RViz 会不停
`Resetting RViz`，画面自然什么都渲染不出来。

注意不要用 `kill -9` 只杀 `ros2 launch`，否则它的子进程会变成孤儿进程继续
运行。正确的清理方式是：

```bash
# 先正常关闭 launch，它会带着子进程一起退出
pkill -INT -f "ros2 launch guardrail_robot"

# 如果还有残留（例如上次被 kill -9 过），再逐个清理
pkill -f "ign gazebo"
pkill -f "parameter_bridge"
pkill -f "robot_state_publisher"
pkill -f "rviz2"
pkill -f "rqt_robot_steering"
```

然后只启动**一份**仿真：

```bash
ros2 launch guardrail_robot sim.launch.py
```

确认时钟只有一个发布者，且日志里没有 `jump back in time`：

```bash
ros2 topic info /clock -v
```

### RViz 中看不到点云

* 点云话题是 `/scan/points`（`sensor_msgs/msg/PointCloud2`），不是 `/scan`
  （`/scan` 是同一次扫描的单线 `LaserScan`）。
* 使用本包的 `rviz/guardrail.rviz` 时，Displays 中已经有一个
  **LiDAR Points** 显示，话题为 `/scan/points`，确认它是勾选状态。
* 如果用的是自己新建的 RViz：添加 **PointCloud2** display，Topic 选
  `/scan/points`，Fixed Frame 选 `odom`。
* 先确认话题真的有数据：

  ```bash
  ros2 topic hz /scan/points
  ```

  正常大约 7–10 Hz。

### 小车没有生成 / 没有 `/odom`

之前异常退出的 Gazebo 进程可能还占用着仿真服务，按上面的清理步骤处理后再
启动。

### 滑块动了但小车不动

检查 `/cmd_vel` 的发布者类型是否为 `geometry_msgs/msg/Twist`：

```bash
ros2 topic info /cmd_vel -v
```

### 转弯不灵 / 原地转不动

四轮差速转向靠轮胎横向搓地完成转向，横向摩擦不能和纵向抓地一样大。
本包在 `urdf/wheels.xacro` 中设置了：

* `mu1`（纵向抓地）= 1.5
* `mu2`（横向搓地）= 0.2
* 轮端力矩上限 = 1500 Nm

另外有一个容易踩的坑：sdformat 的 URDF 解析器只有在 **collision 名称以 link
名称开头**（或为空）时，才会应用 `<gazebo reference>` 里的
`mu1` / `mu2` / `fdir1` / `kp` / `kd`。如果以后修改 collision 名字，请保留
`front_left_wheel_link_...` 这种前缀；否则摩擦参数会被静默忽略，车轮又回到
默认摩擦，转向会非常迟钝。

如果还想更灵敏，可以调大 `urdf/gazebo.xacro` 中的
`max_angular_velocity` / `max_angular_acceleration`，以及
`config/rqt_robot_steering.ini` 中的 `vw_max` / `vw_min`。
