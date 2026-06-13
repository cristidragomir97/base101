#!/usr/bin/env python3
"""base101 Nav2 stack: planner, controller, bt_navigator, velocity smoother.

The navigation half of the robot's autonomy, up for the whole session.
Independent of base101_slam by design (own lifecycle manager, no package
dependency): it consumes /map and the map->odom TF from WHATEVER
publishes them, and starts fine before they exist.

No behavior_server: recovery decisions live in the Python mission layer
(robocore). The BT is plan/follow/fail — Nav2 returns ABORTED on any
failure and the bridge surfaces NavStatus(phase="stuck"). The bridge
clears both costmaps before each goal via their clear services.

cmd_vel topology: controller -> cmd_vel_raw -> velocity_smoother ->
cmd_vel_nav -> twist_mux (priority 10, below teleop).

    ros2 launch base101_nav nav.launch.py use_sim_time:=true
    ros2 launch base101_nav nav.launch.py use_sim_time:=true rviz:=false  # headless

Opens RViz by default (rviz:=false to suppress); the standalone
rviz.launch.py is still there if you want RViz without the stack.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('base101_nav')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    autostart = LaunchConfiguration('autostart').perform(context) == 'true'
    rviz = LaunchConfiguration('rviz').perform(context) == 'true'

    config = {name: os.path.join(pkg_dir, 'config', f'{name}.yaml')
              for name in ('planner', 'controller', 'bt_navigator',
                           'velocity_smoother', 'costmap')}
    bt_dir = os.path.join(pkg_dir, 'behavior_trees')

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[config['planner'], config['costmap'],
                    {'use_sim_time': use_sim_time}],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[config['controller'], config['costmap'],
                    {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_raw')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            config['bt_navigator'],
            {
                'use_sim_time': use_sim_time,
                'default_nav_to_pose_bt_xml':
                    os.path.join(bt_dir, 'nav_to_pose.xml'),
                'default_nav_through_poses_bt_xml':
                    os.path.join(bt_dir, 'nav_through_poses.xml'),
            },
        ],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[config['velocity_smoother'],
                    {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_raw'),
                    ('cmd_vel_smoothed', 'cmd_vel_nav')],
    )

    lifecycle_manager = TimerAction(
        period=3.0,
        actions=[Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_nav',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'bond_timeout': 30.0,
                'node_names': ['planner_server', 'controller_server',
                               'bt_navigator', 'velocity_smoother'],
            }],
        )],
    )

    nodes = [planner_server, controller_server, bt_navigator,
             velocity_smoother, lifecycle_manager]

    if rviz:
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',     # rviz is chatty; keep its spew out of the console
            arguments=['-d', os.path.join(pkg_dir, 'config', 'nav.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start lifecycle nodes',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Open RViz with the nav/SLAM display config '
                        '(rviz:=false for headless runs).',
        ),
        OpaqueFunction(function=_setup),
    ])
