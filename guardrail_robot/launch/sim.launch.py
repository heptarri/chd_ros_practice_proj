#!/usr/bin/env python3
"""Start Gazebo, spawn the robot, start the bridge, RViz and rqt steering."""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("guardrail_robot")
    world_file = os.path.join(pkg_share, "worlds", "highway.sdf")
    xacro_file = os.path.join(pkg_share, "urdf", "guardrail_bot.urdf.xacro")
    bridge_file = os.path.join(pkg_share, "config", "bridge.yaml")
    rviz_file = os.path.join(pkg_share, "rviz", "guardrail.rviz")

    # Expand xacro at launch time.  This avoids shell quoting problems with
    # paths that contain spaces.
    robot_description = xacro.process_file(xacro_file).toxml()

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("rviz")
    start_teleop = LaunchConfiguration("teleop")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": ["-r -v 4 ", world_file],
            "on_exit_shutdown": "true",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "guardrail_bot",
            "-topic", "robot_description",
            # base_footprint is the wheel-contact plane.  The right-hand lane
            # centre at the start of the bend is y=-1.875 m, the road heading
            # is +x and the asphalt top is z=0.02 m.
            "-x", "0.0",
            "-y", "-1.875",
            "-z", "0.03",
            "-R", "0.0",
            "-P", "0.0",
            "-Y", "0.0",
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"config_file": bridge_file}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_file],
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
        condition=IfCondition(start_rviz),
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "teleop.launch.py")
        ),
        condition=IfCondition(start_teleop),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true",
                description="Use the Gazebo clock."),
            DeclareLaunchArgument(
                "rviz", default_value="true",
                description="Start RViz together with Gazebo."),
            DeclareLaunchArgument(
                "teleop", default_value="true",
                description="Start rqt_robot_steering to drive the robot."),
            gz_sim,
            robot_state_publisher,
            spawn_robot,
            bridge,
            rviz,
            teleop,
        ]
    )
