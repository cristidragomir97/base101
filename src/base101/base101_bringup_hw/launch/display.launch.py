#!/usr/bin/env python3
"""RViz-only view of base101, with joint sliders. No sim, no hardware.

Replaces the two per-variant display launches (base101_simple_description and
base101_arm_description), which differed only in which URDF they loaded — now
the `arm` argument.

    ros2 launch base101_bringup_hw display.launch.py
    ros2 launch base101_bringup_hw display.launch.py arm:=true
    ros2 launch base101_bringup_hw display.launch.py gui:=false camera:=oak_d

simulator:=none, so robot_state_publisher sees a pure URDF with no
ros2_control blocks or per-simulator extension tags.
"""

import os
import re

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _configured_tool():
    """The end-effector the mod101 configurator last saved.

    Falls back to the macro's own default when the mod101 underlay isn't
    sourced — which is fine, because arm:=false never reads it.
    """
    try:
        cfg = os.path.join(get_package_share_directory('mod101_description'),
                           'urdf', 'mod101_config.xacro')
        m = re.search(r'<xacro:arg\s+name="tool"\s+default="([^"]+)"',
                      open(cfg).read())
        return m.group(1) if m else 'jaws'
    except Exception:
        return 'jaws'


def _setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    arm = arg('arm') == 'true'
    pkg = get_package_share_directory('base101_description')

    mappings = {'simulator': 'none', 'camera': arg('camera'),
                'arm': str(arm).lower()}
    if arm:
        mappings['arm_tool'] = arg('arm_tool')
    robot_description = xacro.process_file(
        os.path.join(pkg, 'urdf', 'base101.xacro'), mappings=mappings).toxml()

    show_gui = LaunchConfiguration('gui')

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            condition=UnlessCondition(show_gui),
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
        ),
        Node(
            condition=IfCondition(show_gui),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg, 'config', 'display.rviz')],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arm', default_value='false', choices=['true', 'false'],
            description='Show one mod101 arm on the deck (needs the mod101 '
                        'underlay sourced).'),
        DeclareLaunchArgument(
            'arm_tool', default_value=_configured_tool(),
            description='mod101 end-effector (mod101_tool_<name>).'),
        DeclareLaunchArgument(
            'gui', default_value='true', choices=['true', 'false'],
            description='joint_state_publisher_gui (sliders) instead of the '
                        'plain joint_state_publisher.'),
        DeclareLaunchArgument(
            'camera', default_value='realsense', choices=['realsense', 'oak_d'],
            description='Depth module on the front bracket.'),
        OpaqueFunction(function=_setup),
    ])
