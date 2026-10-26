#!/usr/bin/env python3
"""Start rqt_robot_steering pre-configured for the guardrail robot.

The Gazebo diff-drive plugin of ``guardrail_bot`` listens on ``/cmd_vel`` and
expects ``geometry_msgs/msg/Twist``.  rqt_robot_steering stores the topic name
and the "stamped" checkbox in ``~/.config/ros.org/rqt_gui.ini``; a value left
over from another robot (or the default stamped checkbox) makes the sliders
publish to an unrelated topic or publish ``TwistStamped``, in which case the
bridge never forwards the command and the robot does not move.

This launch file starts rqt_robot_steering with a private configuration
directory that already contains the right settings (``/cmd_vel``, unstamped,
10 m/s / 2 rad/s slider range), so the controls work without touching the
user's global rqt configuration.  It uses the small wrapper in
``scripts/rqt_robot_steering_guardrail`` to work around a float/int bug in
rqt_robot_steering 1.0.3 that otherwise keeps the sliders pinned to +/-1 m/s.

Usage:
    ros2 launch guardrail_robot teleop.launch.py
"""

import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def _prepare_rqt_config() -> str:
    """Copy the packaged rqt settings to a writable per-user directory."""
    pkg_share = get_package_share_directory("guardrail_robot")
    source = os.path.join(pkg_share, "config", "rqt_robot_steering.ini")

    config_home = os.path.join(os.path.expanduser("~"), ".config", "guardrail_robot")
    target_dir = os.path.join(config_home, "ros.org")
    os.makedirs(target_dir, exist_ok=True)
    shutil.copyfile(source, os.path.join(target_dir, "rqt_gui.ini"))
    return config_home


def generate_launch_description() -> LaunchDescription:
    config_home = _prepare_rqt_config()

    return LaunchDescription(
        [
            # Only affects the processes started by this launch description.
            SetEnvironmentVariable("XDG_CONFIG_HOME", config_home),
            Node(
                package="guardrail_robot",
                executable="rqt_robot_steering_guardrail",
                output="screen",
            ),
        ]
    )
