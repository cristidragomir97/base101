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
    arm_tool  <name>            mod101 end-effector (default: the configurator's)
    world     <path or name>    .sdf world (default sticky_floor.sdf)
"""

import os
import re

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



def _configured_tool():
    """The end-effector the mod101 configurator last saved.

    Used as the launch default so `ros2 launch` agrees with the configurator
    rather than pinning one tool. `arm_tool:=parallel` still overrides, and if
    the mod101 underlay isn't sourced (arm:=false builds fine without it) this
    falls back to the macro's own default.
    """
    try:
        cfg = os.path.join(get_package_share_directory('mod101_description'),
                           'urdf', 'mod101_config.xacro')
        m = re.search(r'<xacro:arg\s+name="tool"\s+default="([^"]+)"', open(cfg).read())
        return m.group(1) if m else 'jaws'
    except Exception:
        return 'jaws'

def _setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    rosboard_port = LaunchConfiguration('rosboard_port').perform(context)
    arm = LaunchConfiguration('arm').perform(context) == 'true'
    arm_tool = LaunchConfiguration('arm_tool').perform(context)
    arm_control = LaunchConfiguration('arm_control').perform(context)

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
    #
    # Both variants of each controller claim the same joints, so exactly one
    # may be active: `sliders` gives the Float64MultiArray position controllers
    # the web UIs publish to, `moveit` gives the FollowJointTrajectory ones
    # move_group needs. base101_arm_moveit_config's demo launch passes moveit.
    post_jsb_spawners = [spawn_diff_drive]
    if arm:
        suffix = '_trajectory_controller' if arm_control == 'moveit' else '_controller'
        arm_controllers = ['arm' + suffix]
        if arm_tool != 'none':
            arm_controllers += ['gripper' + suffix]
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
        # Intrinsics. ros_gz_image bridges the image stream only, so without
        # this the wrist camera publishes pixels with no camera_info and
        # nothing can project them into 3D — robocore_agent reports it as
        # "camera has no intrinsics". Its own node rather than an entry in
        # base101_gazebo's gz_ros_bridge.yaml, because that config is shared
        # with the armless `simple` variant, which has no wrist camera.
        actions.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='arm_wrist_camera_info_bridge',
            arguments=['/arm_wrist_camera/camera_info@sensor_msgs/msg/'
                       'CameraInfo[gz.msgs.CameraInfo'],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm', default_value='true',
                              choices=['true', 'false'],
                              description='Mount one mod101 arm (needs mod101 underlay).'),
        DeclareLaunchArgument(
            'arm_tool', default_value=_configured_tool(),
                              description='mod101 end-effector (mod101_tool_<name>).'),
        DeclareLaunchArgument('world', default_value='sticky_floor.sdf',
                              description='SDF world (name in base101_gazebo/worlds or absolute path).'),
        DeclareLaunchArgument('arm_control', default_value='sliders',
                              choices=['sliders', 'moveit'],
                              description='Arm controllers to spawn: sliders = position '
                                          '(Float64MultiArray, what the web UIs drive), '
                                          'moveit = trajectory (FollowJointTrajectory).'),
        DeclareLaunchArgument('rosboard', default_value='true',
                              choices=['true', 'false'],
                              description='Run rosboard web dashboard.'),
        DeclareLaunchArgument('rosboard_port', default_value='8888',
                              description='HTTP/WS port for rosboard.'),
        OpaqueFunction(function=_setup),
    ])
