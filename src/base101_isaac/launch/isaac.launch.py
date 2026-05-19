#!/usr/bin/env python3
"""Bring up a base101 variant in Isaac Sim with the ROS2 bridge.

The runner script (scripts/run_isaac.py) owns the simulation loop and the
OmniGraph bridge. This launch wraps it with the same ROS-side scaffolding
the Gazebo/MuJoCo launches use (robot_state_publisher, twist_mux, rosboard)
so the rest of the stack (Nav2, slam_toolbox, joystick teleop) is unaware
of which simulator is underneath.

Why subprocess and not a Node action: Isaac Sim must run inside its own Kit
interpreter, bootstrapped by `from isaacsim import SimulationApp`. Running
it as an ament Node entry_point would risk loading rclpy + Kit in the wrong
order. ExecuteProcess keeps it isolated.

Launch args:
    variant    simple | pro
    scene      Optional .usd scene path; empty → built-in ground plane.
    headless   Run Kit without a viewport (useful for CI).
"""

import os
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Wheel geometry per variant. Mirrors base101_control/config/controllers.*.sim.yaml
# — keep in sync if those change.
WHEEL_GEOMETRY = {
    'simple': {'radius': 0.0363,  'separation': 0.2886, 'max_lin': 1.0, 'max_ang': 2.0},
    'pro':    {'radius': 0.05035, 'separation': 0.38,   'max_lin': 1.5, 'max_ang': 2.5},
}


def _setup(context, *args, **kwargs):
    variant       = LaunchConfiguration('variant').perform(context)
    scene         = LaunchConfiguration('scene').perform(context)
    headless      = LaunchConfiguration('headless').perform(context) == 'true'
    rosboard_port = LaunchConfiguration('rosboard_port').perform(context)

    pkg_description = get_package_share_directory('base101_description')
    pkg_control     = get_package_share_directory('base101_control')
    pkg_isaac       = get_package_share_directory('base101_isaac')

    urdf_file = os.path.join(pkg_description, 'urdf', 'base101.xacro')

    # Isaac's URDF importer reads from a file path, not a string. Cache the
    # processed URDF to a tmp file the runner can open. Same path is also
    # used for robot_state_publisher so visual/TF tree matches the sim.
    robot_description = xacro.process_file(
        urdf_file, mappings={'variant': variant, 'simulator': 'isaac'}
    ).toxml()
    urdf_tmp = os.path.join(
        tempfile.gettempdir(), f'base101_isaac_{variant}.urdf',
    )
    with open(urdf_tmp, 'w', encoding='utf-8') as f:
        f.write(robot_description)

    geom = WHEEL_GEOMETRY[variant]

    runner_script = os.path.join(pkg_isaac, 'scripts', 'run_isaac.py')

    isaac_runner = ExecuteProcess(
        cmd=[
            'python3', runner_script,
            '--urdf', urdf_tmp,
            '--variant', variant,
            '--scene', scene,
            '--wheel-radius', str(geom['radius']),
            '--wheel-separation', str(geom['separation']),
            '--max-linear',  str(geom['max_lin']),
            '--max-angular', str(geom['max_ang']),
            '--cmd-topic', '/diff_drive_controller/cmd_vel',
        ] + (['--headless'] if headless else []),
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # twist_mux gives Nav2 / joystick / keyboard a place to fight, and its
    # cmd_vel_out is the topic the Isaac runner subscribes to (see
    # --cmd-topic above). This keeps the topology identical to gazebo.
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

    return [
        isaac_runner,
        robot_state_publisher,
        twist_mux,
        rosboard,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'variant',
            default_value='simple',
            choices=['simple', 'pro'],
        ),
        DeclareLaunchArgument(
            'scene',
            default_value='',
            description='Optional USD scene file. Empty → default ground plane.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            choices=['true', 'false'],
            description='Run Isaac Sim without a viewport window.',
        ),
        DeclareLaunchArgument(
            'rosboard',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'rosboard_port',
            default_value='8888',
        ),
        OpaqueFunction(function=_setup),
    ])
