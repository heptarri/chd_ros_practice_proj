# guardrail_robot

ROS 2 Humble + Gazebo（Fortress / ignition-gazebo6）中的护栏巡检小车仿真。

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

## 用 rqt_robot_steering 遥控

`sim.launch.py` 已经启动了一个配置好的 `rqt_robot_steering` 窗口，直接拖动
**Linear x**（前进/后退）和 **Angular z**（左转/右转）滑块即可，松开滑块前
小车会持续按当前速度行驶，把滑块拖回 0 或点击 **Stop** 停车。

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

如果不想通过 launch 文件启动，也可以直接运行：

```bash
ros2 run rqt_robot_steering rqt_robot_steering
```

然后确认窗口中的 **Topic** 为 `/cmd_vel`，并**取消勾选 stamped**。

## 话题说明

| 方向 | ROS 话题 | 类型 |
| --- | --- | --- |
| 控制输入 | `/cmd_vel` | `geometry_msgs/msg/Twist` |
| 里程计 | `/odom` | `nav_msgs/msg/Odometry` |
| 激光雷达 | `/scan` | `sensor_msgs/msg/LaserScan` |
| 相机 | `/camera/image_raw` | `sensor_msgs/msg/Image` |
| IMU | `/imu` | `sensor_msgs/msg/Imu` |

## 常见问题

* **小车没有生成 / 没有 `/odom`**：之前异常退出的 Gazebo 进程可能还占用着
  仿真服务，先清理再启动：

  ```bash
  pkill -f "ign gazebo"
  ```

* **滑块动了但小车不动**：检查 `/cmd_vel` 的发布者类型是否为
  `geometry_msgs/msg/Twist`：

  ```bash
  ros2 topic info /cmd_vel -v
  ```
