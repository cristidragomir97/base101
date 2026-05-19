#!/usr/bin/env python3
"""Bring up the chosen base101 variant in Gazebo Sim with ros2_control.

The gz_ros2_control plugin (declared inside the description's per-variant
.gazebo file) creates the controller_manager once the model is spawned; this
launch then loads the joint_state_broadcaster and diff_drive_controller from
base101_control/config/controllers.<variant>.sim.yaml.

Launch args:
    variant   simple | pro     which hardware variant to load (default simple)
    world     <path or name>   .sdf world file (default: this package's empty.sdf)
"""

import os

import xacro
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    variant = LaunchConfiguration('variant').perform(context)
    world = LaunchConfiguration('world').perform(context)
    rosboard_port = LaunchConfiguration('rosboard_port').perform(context)

    pkg_description = get_package_share_directory('base101_description')
    pkg_control     = get_package_share_directory('base101_control')
    pkg_gazebo      = get_package_share_directory('base101_gazebo')
    pkg_ros_gz_sim  = get_package_share_directory('ros_gz_sim')

    # Gazebo resolves `package://<pkg>/...` mesh URIs by walking
    # GZ_SIM_RESOURCE_PATH for a directory containing <pkg>/. The description
    # uses file://$(find base101_description)/... so we don't strictly need
    # this, but it doesn't hurt and matches the mod101/LLMy convention.
    resource_dirs = [
        os.path.join(get_package_prefix('base101_description'), 'share'),
        os.path.join(get_package_prefix('base101_control'), 'share'),
    ]
    if os.environ.get('GZ_SIM_RESOURCE_PATH'):
        resource_dirs.append(os.environ['GZ_SIM_RESOURCE_PATH'])
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join(resource_dirs),
    )

    world_file = world if os.path.isabs(world) else os.path.join(
        pkg_gazebo, 'worlds', world
    )
    urdf_file = os.path.join(pkg_description, 'urdf', 'base101.xacro')
    bridge_config = os.path.join(pkg_gazebo, 'config', 'gz_ros_bridge.yaml')

    robot_description = xacro.process_file(
        urdf_file, mappings={'variant': variant, 'simulator': 'gazebo'}
    ).toxml()

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

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    base_camera_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='base_camera_image_bridge',
        arguments=['/base_camera/image_raw'],
        parameters=[{'use_sim_time': True}],
        output='screen',
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

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', f'base101_{variant}',
            '-allow_renaming', 'false',
            '-z', '0.10',
        ],
        output='screen',
    )

    # rosboard — web dashboard at http://<host>:<rosboard_port>/, also serves
    # the bundled Teleop card (Twist publisher to /cmd_vel_joy) under the
    # System nav. Disable with rosboard:=false if you want a quieter launch.
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

    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    spawn_diff_drive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Controllers must wait until the robot is spawned (which is when the
    # gz_ros2_control plugin creates the controller_manager).
    after_spawn = RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot,
        on_exit=[spawn_jsb],
    ))
    after_jsb = RegisterEventHandler(OnProcessExit(
        target_action=spawn_jsb,
        on_exit=[spawn_diff_drive],
    ))

    return [
        gz_resource_path,
        robot_state_publisher,
        gz_sim,
        clock_bridge,
        bridge,
        base_camera_image_bridge,
        twist_mux,
        spawn_robot,
        after_spawn,
        after_jsb,
        rosboard,
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
            'world',
            default_value='sticky_floor.sdf',
            description='SDF world file (name in base101_gazebo/worlds or absolute path).',
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
