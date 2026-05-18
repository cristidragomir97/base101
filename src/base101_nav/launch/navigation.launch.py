#!/usr/bin/env python3
"""
Navigation Launch File for base101 robot.

Full Nav2 stack against a pre-built map. Localization is done by
slam_toolbox in localization mode (replaces AMCL + map_server in one
node — see config/slam_toolbox_localization.yaml).

Map: pass `map:=<base_path>` where <base_path> is the slam_toolbox
serialized prefix WITHOUT extension (slam_toolbox loads
<base_path>.posegraph and <base_path>.data).

Stack:
  - slam_toolbox (localization mode): map + map->odom TF
  - planner_server (SmacPlanner2D)
  - smoother_server (SimpleSmoother) — cleans up jagged planner output
  - controller_server (MPPI)
  - bt_navigator
  - behavior_server (spin, backup, drive_on_heading, assisted_teleop, wait)
  - velocity_smoother
  - lifecycle_manager
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    pkg_dir = get_package_share_directory('base101_nav')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_path = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')

    planner_config = os.path.join(pkg_dir, 'config', 'planner.yaml')
    controller_config = os.path.join(pkg_dir, 'config', 'controller.yaml')
    costmap_config = os.path.join(pkg_dir, 'config', 'costmap.yaml')
    localization_config = os.path.join(pkg_dir, 'config', 'slam_toolbox_localization.yaml')
    bt_config = os.path.join(pkg_dir, 'config', 'bt_navigator.yaml')
    behavior_config = os.path.join(pkg_dir, 'config', 'behavior.yaml')
    velocity_smoother_config = os.path.join(pkg_dir, 'config', 'velocity_smoother.yaml')
    # NOTE: no separate smoother_server. SmacPlanner2D smooths internally
    # (see planner.yaml `smoother:` block). Adding a standalone smoother
    # caused a BT blackboard race that aborted FollowPath every cycle.

    bt_dir = os.path.join(pkg_dir, 'behavior_trees')

    # Default map: slam_toolbox serialized base path, NO extension.
    default_map = os.path.join(os.path.expanduser('~'), '.base101', 'maps', 'home')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation time'),
        DeclareLaunchArgument('map', default_value=default_map,
                              description='slam_toolbox serialized map base path (no extension)'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Automatically start lifecycle nodes'),

        # slam_toolbox in localization mode — owns map and map->odom TF.
        LifecycleNode(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            namespace='',
            output='screen',
            parameters=[
                localization_config,
                {'use_sim_time': use_sim_time},
                {'map_file_name': map_path},
            ]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_config, costmap_config,
                        {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_config, costmap_config,
                        {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'cmd_vel_raw')]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                bt_config,
                {'use_sim_time': use_sim_time},
                {'default_nav_to_pose_bt_xml': os.path.join(bt_dir, 'nav_to_pose.xml')},
                {'default_nav_through_poses_bt_xml': os.path.join(bt_dir, 'nav_through_poses.xml')},
            ]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[behavior_config, costmap_config,
                        {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'cmd_vel_raw')]
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[velocity_smoother_config,
                        {'use_sim_time': use_sim_time}],
            remappings=[
                ('cmd_vel', 'cmd_vel_raw'),
                ('cmd_vel_smoothed', 'cmd_vel_nav'),
            ]
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[
                        {'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'bond_timeout': 20.0},
                        {'node_names': [
                            'slam_toolbox',
                            'planner_server',
                            'controller_server',
                            'bt_navigator',
                            'behavior_server',
                            'velocity_smoother',
                        ]},
                    ]
                ),
            ]
        ),
    ])
