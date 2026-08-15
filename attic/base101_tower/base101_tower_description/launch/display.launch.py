#!/usr/bin/env python3
"""RViz display of base101 + cross tower (+ optional dual mod101 arms).

Loads the manipulation overlay URDF (simulator:=none, rviz-only) and drives the
joints with joint_state_publisher(_gui). Reuses base101_description's display
RViz preset.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    pkg_manip = get_package_share_directory('base101_tower_description')
    pkg_desc = get_package_share_directory('base101_description')
    arms = LaunchConfiguration('arms').perform(context)
    arm_tool = LaunchConfiguration('arm_tool').perform(context)

    xacro_file = os.path.join(pkg_manip, 'urdf', 'base101_tower.xacro')
    robot_urdf = xacro.process_file(
        xacro_file, mappings={'simulator': 'none',
                              'arms': arms, 'arm_tool': arm_tool}
    ).toxml()

    rviz_config_file = os.path.join(pkg_desc, 'config', 'display.rviz')
    show_gui = LaunchConfiguration('gui')

    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher',
             parameters=[{'robot_description': robot_urdf}]),
        Node(condition=UnlessCondition(show_gui),
             package='joint_state_publisher', executable='joint_state_publisher',
             name='joint_state_publisher'),
        Node(condition=IfCondition(show_gui),
             package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             name='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_config_file], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arms', default_value='false',
                              choices=['true', 'false'],
                              description='Mount two mod101 arms (needs mod101 underlay).'),
        DeclareLaunchArgument('arm_tool', default_value='jaws',
                              description='mod101 end-effector.'),
        DeclareLaunchArgument('gui', default_value='True',
                              description='Use joint_state_publisher_gui instead of joint_state_publisher.'),
        OpaqueFunction(function=_launch_setup),
    ])
