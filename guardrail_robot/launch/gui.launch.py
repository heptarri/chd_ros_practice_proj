#!/usr/bin/env python3
"""Start the PyQt5 guardrail control panel (camera + mode + joystick).

The GUI subscribes to the camera and publishes geometry_msgs/Twist on
/cmd_vel, so it replaces rqt_robot_steering for manual driving and can also
run the line follower by itself.  Launch the simulation without the default
teleop window to avoid two /cmd_vel publishers:

    ros2 launch guardrail_robot sim.launch.py teleop:=false
    ros2 launch guardrail_robot gui.launch.py

Or run it directly with:

    ros2 run guardrail_robot guardrail_gui
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    speed = LaunchConfiguration("speed")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "speed", default_value="3.0",
                description="Line-following cruise speed [m/s]."),
            Node(
                package="guardrail_robot",
                executable="guardrail_gui",
                name="guardrail_gui",
                output="screen",
                parameters=[{
                    "speed": ParameterValue(speed, value_type=float),
                }],
            ),
        ]
    )
