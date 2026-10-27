#!/usr/bin/env python3
"""Start the web UI backend (React control panel + ROS bridge).

The server serves the React app and exposes /stream (camera + telemetry) and
/cmd (mode / joystick commands).  Open http://localhost:8000 in a browser.

The simulation should be running first:

    ros2 launch guardrail_robot sim.launch.py teleop:=false
    ros2 launch guardrail_robot webui.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    port = LaunchConfiguration("port")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port", default_value="8000",
                description="Port for the web UI."),
            Node(
                package="guardrail_robot",
                executable="webui_server.py",
                name="webui_server",
                output="screen",
                parameters=[{
                    "port": ParameterValue(port, value_type=int),
                }],
            ),
        ]
    )
