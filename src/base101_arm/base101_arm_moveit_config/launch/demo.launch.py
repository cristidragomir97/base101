#!/usr/bin/env python3
"""Gazebo + move_group for the base101 arm variant, in the right order.

Brings the sim up with the trajectory controllers instead of the slider ones
(`arm_control:=moveit`), then starts move_group once the controllers exist.

    ros2 launch base101_arm_moveit_config demo.launch.py
    ros2 launch base101_arm_moveit_config demo.launch.py arm_tool:=parallel

The delay is the same crude approach mod101's demo uses: move_group needs the
controller_manager to be advertising its FollowJointTrajectory actions before it
builds its execution manager, and there is no clean event to latch onto from a
separate launch file.
"""

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _configured_tool():
    """The end-effector the mod101 configurator last saved."""
    try:
        cfg = os.path.join(get_package_share_directory('mod101_description'),
                           'urdf', 'mod101_config.xacro')
        m = re.search(r'<xacro:arg\s+name="tool"\s+default="([^"]+)"', open(cfg).read())
        return m.group(1) if m else 'jaws'
    except Exception:
        return 'jaws'


def generate_launch_description():
    tool = LaunchConfiguration('arm_tool')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('base101_arm_gazebo'),
            'launch', 'gazebo.launch.py')),
        launch_arguments={
            'arm': 'true',
            'arm_tool': tool,
            # The whole point: spawn arm_trajectory_controller /
            # gripper_trajectory_controller rather than the Float64MultiArray
            # pair the web sliders drive. They claim the same joints.
            'arm_control': 'moveit',
            'world': LaunchConfiguration('world'),
            'rosboard': LaunchConfiguration('rosboard'),
        }.items(),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('base101_arm_moveit_config'),
            'launch', 'move_group.launch.py')),
        launch_arguments={
            'arm_tool': tool,
            'use_sim_time': 'true',
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('arm_tool', default_value=_configured_tool()),
        DeclareLaunchArgument('world', default_value='sticky_floor.sdf'),
        DeclareLaunchArgument('rosboard', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        gazebo,
        TimerAction(period=16.0, actions=[move_group]),
    ])
