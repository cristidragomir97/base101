#!/usr/bin/env python3
"""Bring up the base101 control stack on real hardware.

Starts robot_state_publisher with the hardware overlay URDF, the
controller_manager loaded with controllers.hw.yaml, and spawns the
joint_state_broadcaster + diff_drive_controller. A twist_mux is wired in
front of /diff_drive_controller/cmd_vel so multiple cmd_vel sources can
coexist.

NOTE: the hardware overlay (base101.hardware.xacro) currently uses
mock_components/GenericSystem as a placeholder. Swap that for the real
motor driver's SystemInterface plugin before running on hardware.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _setup(context, *args, **kwargs):
    controllers_cfg = PathJoinSubstitution([
        FindPackageShare('base101_simple_control'), 'config', 'controllers.hw.yaml',
    ])
    twist_mux_cfg = PathJoinSubstitution([
        FindPackageShare('base101_control'), 'config', 'twist_mux.yaml',
    ])
    hardware_xacro = PathJoinSubstitution([
        FindPackageShare('base101_simple_control'), 'urdf', 'base101_simple.hardware.xacro',
    ])

    robot_description = ParameterValue(
        Command(['xacro ', hardware_xacro, ' simulator:=none']),
        value_type=str,
    )

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': False},
            ],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': False},
                controllers_cfg,
            ],
        ),
        TimerAction(
            period=3.0,
            actions=[Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster',
                           '--controller-manager', '/controller_manager'],
                output='screen',
            )],
        ),
        TimerAction(
            period=5.0,
            actions=[Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller',
                           '--controller-manager', '/controller_manager'],
                output='screen',
            )],
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[twist_mux_cfg, {'use_sim_time': False}],
            remappings=[('cmd_vel_out', '/diff_drive_controller/cmd_vel')],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=_setup),
    ])
