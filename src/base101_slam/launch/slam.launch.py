#!/usr/bin/env python3
"""base101 SLAM stack: EKF odometry fusion + slam_toolbox.

Launches the localization half of the robot's autonomy: it stays up for
the whole session, starts in mapping mode, and the robocore bridge
switches it to localization at runtime via slam_toolbox services
(serialize_map / deserialize_map) — modes are service calls, never
process restarts.

Independent of base101_nav by design: each stack has its own lifecycle
manager, so Nav2 can start, run and die without slam_toolbox and vice
versa. The only coupling is the /map topic and the map->odom TF this
stack publishes.

    ros2 launch base101_slam slam.launch.py use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('base101_slam')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    autostart = LaunchConfiguration('autostart').perform(context) == 'true'
    slam_config = LaunchConfiguration('slam_config').perform(context)
    if not slam_config:
        slam_config = os.path.join(pkg_dir, 'config', 'slam_toolbox.yaml')

    # Sim and real robot fuse different sensors (and the sim config
    # tolerates the missing ones); pick by use_sim_time.
    ekf_config = os.path.join(
        pkg_dir, 'config', 'ekf.sim.yaml' if use_sim_time else 'ekf.yaml')

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odometry/filtered')],
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, {'use_sim_time': use_sim_time}],
    )

    # slam_toolbox is a LifecycleNode that does NOT self-activate; it
    # sits in `unconfigured` until something drives configure->activate.
    # So a lifecycle manager is required to bring it up — but with
    # bond_timeout 0.0 to DISABLE the bond heartbeat. The bond misfires
    # under sim time (manager reports "connected" then "no heartbeat for
    # 30000 ms" 200 ms later, looping deactivate/reactivate forever);
    # disabling it keeps the one useful job (autostart) without the
    # broken watchdog. Crash supervision is the robocore bridge's job.
    # EKF is not a lifecycle node and is not managed.
    lifecycle_manager = TimerAction(
        period=3.0,
        actions=[Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'bond_timeout': 0.0,        # disable the bond heartbeat
                'node_names': ['slam_toolbox'],
            }],
        )],
    )

    return [ekf, slam_toolbox, lifecycle_manager]


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
            description='Automatically configure+activate slam_toolbox',
        ),
        DeclareLaunchArgument(
            'slam_config',
            default_value='',
            description='Override the slam_toolbox config file',
        ),
        OpaqueFunction(function=_setup),
    ])
