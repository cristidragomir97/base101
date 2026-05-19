#!/usr/bin/env python3
"""Bring up a base101 variant in MuJoCo with mujoco_ros2_control.

The mujoco_ros2_control node loads the MJCF scene, hosts the
controller_manager, and exposes joint command/state interfaces named by the
MuJoCo joint names (which must match base101.mujoco.ros2control). Cameras
declared in the MJCF are picked up automatically by mujoco_cameras.cpp and
published to ROS image topics.

Lidar is not yet supported by mujoco_ros2_control upstream, so we run
base101_mujoco.lidar_bridge alongside the sim. It loads a parallel copy of
the same MJCF, watches /tf for the lidar frame, and ray-casts against the
static world geoms.

Launch args:
    variant   simple | pro       Which hardware variant to load.
    scene     <path or basename> MJCF scene under base101_mujoco/scenes/, or
                                 an absolute path to a custom MJCF.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    variant = LaunchConfiguration('variant').perform(context)
    scene = LaunchConfiguration('scene').perform(context)
    rosboard_port = LaunchConfiguration('rosboard_port').perform(context)

    pkg_description = get_package_share_directory('base101_description')
    pkg_control     = get_package_share_directory('base101_control')
    pkg_mujoco      = get_package_share_directory('base101_mujoco')

    # Empty default → variant-specific scene; otherwise honor the user value.
    if not scene:
        scene = f'base101_{variant}.xml'
    scene_path = scene if os.path.isabs(scene) else os.path.join(
        pkg_mujoco, 'scenes', scene
    )
    if not scene_path.endswith('.xml'):
        scene_path = f'{scene_path}.xml'

    urdf_file = os.path.join(pkg_description, 'urdf', 'base101.xacro')
    controllers_yaml = os.path.join(
        pkg_control, 'config', f'controllers.{variant}.sim.yaml'
    )

    robot_description = xacro.process_file(
        urdf_file, mappings={'variant': variant, 'simulator': 'mujoco'}
    ).toxml()
    robot_description_param = {'robot_description': robot_description}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param, {'use_sim_time': True}],
    )

    # mujoco_ros2_control owns the simulation loop: it loads the MJCF, hosts
    # the controller_manager from controllers_yaml, and publishes any cameras
    # declared in the MJCF.
    mujoco_node = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description_param,
            controllers_yaml,
            {'mujoco_model_path': scene_path},
            {'use_sim_time': True},
        ],
    )

    # Lidar bridge: parallel mjData ray casting on the static world.
    lidar_bridge = Node(
        package='base101_mujoco',
        executable='lidar_bridge',
        name='mujoco_lidar_bridge',
        output='screen',
        parameters=[{
            'mujoco_model_path': scene_path,
            'lidar_frame_id': 'lidar_frame',
            'world_frame_id': 'odom',
            'samples': 360,
            'range_min': 0.12,
            'range_max': 12.0,
            'update_rate': 10.0,
            'use_sim_time': True,
        }],
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            os.path.join(pkg_control, 'config', 'twist_mux.yaml'),
            {'use_sim_time': True},
        ],
        remappings=[('cmd_vel_out', '/diff_drive_controller/cmd_vel')],
    )

    rosboard = Node(
        package='rosboard',
        executable='rosboard_node',
        name='rosboard',
        output='screen',
        parameters=[{
            'port': int(rosboard_port),
            'use_sim_time': True,
        }],
        condition=IfCondition(LaunchConfiguration('rosboard')),
    )

    # Controllers are loaded via the CLI once the controller_manager is up
    # inside mujoco_ros2_control. ExecuteProcess matches the upstream demo
    # launch pattern.
    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen',
    )
    load_diff_drive = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_controller'],
        output='screen',
    )

    return [
        robot_state_publisher,
        mujoco_node,
        twist_mux,
        lidar_bridge,
        rosboard,
        RegisterEventHandler(OnProcessStart(
            target_action=mujoco_node, on_start=[load_jsb],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=load_jsb, on_exit=[load_diff_drive],
        )),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'variant',
            default_value='simple',
            choices=['simple', 'pro'],
            description='base101 hardware variant (simple or pro).',
        ),
        DeclareLaunchArgument(
            'scene',
            default_value='',
            description='MJCF scene file (basename under base101_mujoco/scenes '
                        'or absolute path). Empty → base101_<variant>.xml.',
        ),
        DeclareLaunchArgument(
            'rosboard',
            default_value='true',
            choices=['true', 'false'],
            description='Run rosboard web dashboard + teleop card alongside the sim.',
        ),
        DeclareLaunchArgument(
            'rosboard_port',
            default_value='8888',
            description='HTTP/WS port for rosboard.',
        ),
        OpaqueFunction(function=_setup),
    ])
