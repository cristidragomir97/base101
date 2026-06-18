#!/usr/bin/env python3
"""Bring up base101 + a single mod101 arm in Gazebo Sim.

This is the single-arm manipulation overlay's sim launch. It mirrors
base101_gazebo/launch/gazebo.launch.py but loads the overlay URDF
(base101_arm/urdf/base101_arm.xacro) and additionally spawns the arm
controllers and bridges the wrist camera. The arm controller params live in
this package (controllers.arm.yaml) and are loaded onto the gz_ros2_control
controller_manager at spawn time via the spawner's --param-file, so the core
base101 packages stay arm-free.

Args:
    variant   simple           hardware variant (default simple)
    arm       true | false      mount one mod101 arm (needs mod101 underlay)
    arm_tool  <name>            mod101 end-effector (default jaws)
    world     <path or name>    .sdf world (default sticky_floor.sdf)
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
    world = LaunchConfiguration('world').perform(context)
    rosboard_port = LaunchConfiguration('rosboard_port').perform(context)
    arm = LaunchConfiguration('arm').perform(context) == 'true'
    arm_tool = LaunchConfiguration('arm_tool').perform(context)

    pkg_manip       = get_package_share_directory('base101_arm_description')
    pkg_control     = get_package_share_directory('base101_control')
    pkg_gazebo      = get_package_share_directory('base101_gazebo')
    pkg_ros_gz_sim  = get_package_share_directory('ros_gz_sim')

    # Gazebo resolves package:// mesh URIs (the mod101 tool meshes use them) by
    # walking GZ_SIM_RESOURCE_PATH for a dir containing <pkg>/.
    resource_dirs = [
        os.path.join(get_package_prefix('base101_description'), 'share'),
        os.path.join(get_package_prefix('base101_arm_description'), 'share'),
    ]
    if arm:
        resource_dirs += [
            os.path.join(get_package_prefix('mod101_description'), 'share'),
            os.path.join(get_package_prefix(f'mod101_tool_{arm_tool}'), 'share'),
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
    urdf_file = os.path.join(pkg_manip, 'urdf', 'base101_arm.xacro')
    bridge_config = os.path.join(pkg_gazebo, 'config', 'gz_ros_bridge.yaml')

    robot_description = xacro.process_file(
        urdf_file, mappings={
            'simulator': 'gazebo',
            'arm': str(arm).lower(),
            'arm_tool': arm_tool,
        }
    ).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
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
    base_camera_depth_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='base_camera_depth_bridge',
        arguments=['/base_camera/depth_image'],
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
        arguments=['-topic', 'robot_description',
                   '-name', 'base101_arm',
                   '-allow_renaming', 'false', '-z', '0.10'],
        output='screen',
    )

    rosboard = Node(
        package='rosboard',
        executable='rosboard_node',
        name='rosboard',
        output='screen',
        parameters=[{'port': int(rosboard_port), 'use_sim_time': True}],
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

    # Arm controllers: their params are already loaded onto the
    # controller_manager by the gz_ros2_control plugin (base101_arm_control/
    # config/controllers.sim.yaml), so we just spawn them by name.
    post_jsb_spawners = [spawn_diff_drive]
    if arm:
        arm_controllers = ['arm_controller']
        if arm_tool != 'none':
            arm_controllers += ['gripper_controller']
        post_jsb_spawners += [Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager'],
            output='screen',
        ) for name in arm_controllers]

    after_spawn = RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot, on_exit=[spawn_jsb]))
    after_jsb = RegisterEventHandler(OnProcessExit(
        target_action=spawn_jsb, on_exit=post_jsb_spawners))

    actions = [
        gz_resource_path,
        robot_state_publisher,
        gz_sim,
        clock_bridge,
        bridge,
        base_camera_image_bridge,
        base_camera_depth_bridge,
        twist_mux,
        spawn_robot,
        after_spawn,
        after_jsb,
        rosboard,
    ]
    if arm:
        actions.append(Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='arm_wrist_camera_image_bridge',
            arguments=['/arm_wrist_camera/image_raw'],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm', default_value='true',
                              choices=['true', 'false'],
                              description='Mount one mod101 arm (needs mod101 underlay).'),
        DeclareLaunchArgument('arm_tool', default_value='jaws',
                              description='mod101 end-effector (mod101_tool_<name>).'),
        DeclareLaunchArgument('world', default_value='sticky_floor.sdf',
                              description='SDF world (name in base101_gazebo/worlds or absolute path).'),
        DeclareLaunchArgument('rosboard', default_value='true',
                              choices=['true', 'false'],
                              description='Run rosboard web dashboard.'),
        DeclareLaunchArgument('rosboard_port', default_value='8888',
                              description='HTTP/WS port for rosboard.'),
        OpaqueFunction(function=_setup),
    ])
