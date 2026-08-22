#!/usr/bin/env python3
"""base101 in Gazebo Sim — the whole robot, one launch.

Replaces base101_simple_gazebo and base101_arm_gazebo, which were ~90%
identical and differed only in whether an arm was on the deck. That is now
the `arm` argument. The argument contract matches base101_bringup_hw's, so
what you type at the sim is what you type at the robot (minus `world` and
`camera`, which only mean something here).

    ros2 launch base101_bringup_gazebo sim.launch.py
    ros2 launch base101_bringup_gazebo sim.launch.py arm:=true
    ros2 launch base101_bringup_gazebo sim.launch.py arm:=true moveit:=true
    ros2 launch base101_bringup_gazebo sim.launch.py nav:=false rosboard:=false
    ros2 launch base101_bringup_gazebo sim.launch.py agent:=false        # no robocore
    ros2 launch base101_bringup_gazebo sim.launch.py world:=empty.sdf rviz:=true

Startup order, which is the whole reason this file is not a flat list:

    gz_sim + robot_state_publisher + bridges     (immediately)
      -> spawn model into the world              (creates controller_manager,
                                                  via the gz_ros2_control
                                                  plugin in base101.xacro)
        -> joint_state_broadcaster
          -> diff_drive_controller [+ arm controllers]
            -> +5 s: slam + nav2 [+ move_group] + robocore agent

The autonomy tail is delayed rather than raced: slam_toolbox wants /scan and
the odom->base_link TF (diff_drive_controller's) before it will build a map.

Nav2 and SLAM are launched together on purpose. Nav2 alone is inert —
planner_server blocks in `Activating` forever waiting for a `map` frame that
only slam publishes, which reads from the outside as "nav doesn't launch".
They remain independent packages with their own lifecycle managers; this
file only decides that they start together.

See docs/bringup-restructure.md.
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
    TimerAction,
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
    the mod101 underlay isn't sourced (arm:=false runs fine without it) this
    falls back to the macro's own default.
    """
    try:
        cfg = os.path.join(get_package_share_directory('mod101_description'),
                           'urdf', 'mod101_config.xacro')
        m = re.search(r'<xacro:arg\s+name="tool"\s+default="([^"]+)"',
                      open(cfg).read())
        return m.group(1) if m else 'jaws'
    except Exception:
        return 'jaws'


def _stack(package, launch_file, **launch_args):
    """Include a stack launch (slam / nav / move_group) with sim time on."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package), 'launch', launch_file)),
        launch_arguments={'use_sim_time': 'true', **launch_args}.items(),
    )


# Where to look for robocore profiles when `profile:=` is not given. They are
# owned by the robocore engine repo, not this workspace (docker-compose,
# simulation.yaml and the course notebooks all reference engine/profiles/),
# so this resolves a path rather than shipping a copy that would drift.
PROFILE_DIRS = (
    '/profiles',                                    # container mount
    os.path.expanduser('~/Work/bpe/engine/profiles'),
    os.path.expanduser('~/bpe/engine/profiles'),
)


def resolve_profile(profile, arm):
    """Absolute path to the robocore profile for this configuration.

    Explicit `profile:=` wins; then $ROBOCORE_PROFILE; then the first
    candidate directory that has the right file for the arm/armless split.
    """
    if profile:
        if not os.path.isfile(profile):
            raise RuntimeError(f'profile:={profile} does not exist')
        return profile
    env = os.environ.get('ROBOCORE_PROFILE')
    if env:
        if not os.path.isfile(env):
            raise RuntimeError(f'$ROBOCORE_PROFILE={env} does not exist')
        return env
    name = 'base101_arm.yaml' if arm else 'base101.yaml'
    for d in PROFILE_DIRS:
        candidate = os.path.join(d, name)
        if os.path.isfile(candidate):
            return candidate
    raise RuntimeError(
        f'no robocore profile found: looked for {name} in '
        f'{", ".join(PROFILE_DIRS)}. Pass profile:=/path/to.yaml, set '
        '$ROBOCORE_PROFILE, or launch with agent:=false.')


def _setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    world = arg('world')
    camera = arg('camera')
    arm = arg('arm') == 'true'
    arm_tool = arg('arm_tool')
    moveit = arg('moveit') == 'true'
    # move_group executes over FollowJointTrajectory, so moveit:=true implies
    # the trajectory controllers whatever arm_control says. Both variants of
    # each controller claim the same joints, so exactly one may be active.
    arm_control = 'moveit' if moveit else arg('arm_control')
    nav = arg('nav') == 'true'
    slam = arg('slam') == 'true'
    agent = arg('agent') == 'true'
    rviz = arg('rviz')
    rosboard_port = arg('rosboard_port')

    if moveit and not arm:
        raise RuntimeError(
            'moveit:=true needs an arm to plan for — pass arm:=true too.')

    pkg_description = get_package_share_directory('base101_description')
    pkg_control     = get_package_share_directory('base101_control')
    pkg_worlds      = get_package_share_directory('base101_worlds')
    pkg_ros_gz_sim  = get_package_share_directory('ros_gz_sim')

    # Gazebo resolves `package://<pkg>/...` mesh URIs by walking
    # GZ_SIM_RESOURCE_PATH for a directory containing <pkg>/. The chassis uses
    # file://$(find base101_description)/... so it doesn't strictly need this,
    # but the mod101 tool meshes do use package://.
    resource_dirs = [
        os.path.join(get_package_prefix('base101_description'), 'share'),
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
        pkg_worlds, 'worlds', world)
    bridge_config = os.path.join(pkg_worlds, 'config', 'gz_ros_bridge.yaml')

    # One description for every configuration. arm_tool is only passed when
    # there is an arm: with arm:=false the mod101 config that declares it is
    # never included, and passing an undeclared mapping is a xacro error.
    mappings = {'simulator': 'gazebo', 'camera': camera, 'arm': str(arm).lower()}
    if arm:
        mappings['arm_tool'] = arm_tool
    robot_description = xacro.process_file(
        os.path.join(pkg_description, 'urdf', 'base101.xacro'),
        mappings=mappings,
    ).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
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

    # ros_gz_image bridges image streams; the matching camera_info comes from
    # the parameter_bridge config above. Depth is 32FC1 metres, consumed by
    # the robocore agent for deprojection/clouds.
    image_bridges = [
        Node(package='ros_gz_image', executable='image_bridge',
             name=f'{name}_bridge', arguments=[topic],
             parameters=[{'use_sim_time': True}], output='screen')
        for name, topic in (
            ('base_camera_image', '/base_camera/image_raw'),
            ('base_camera_depth', '/base_camera/depth_image'),
        )
    ]

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[os.path.join(pkg_control, 'config', 'twist_mux.yaml'),
                    {'use_sim_time': True}],
        remappings=[('cmd_vel_out', '/diff_drive_controller/cmd_vel')],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'base101',
                   '-allow_renaming', 'false',
                   '-z', '0.10'],
        output='screen',
    )

    # rosboard — web dashboard at http://<host>:<rosboard_port>/, also serves
    # the bundled Teleop card (Twist publisher to /cmd_vel_joy) under the
    # System nav. rosboard:=false for a quieter launch.
    rosboard = Node(
        package='rosboard',
        executable='rosboard_node',
        name='rosboard',
        output='screen',
        parameters=[{'port': int(rosboard_port), 'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rosboard')),
    )

    def spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager'],
            output='screen',
        )

    spawn_jsb = spawner('joint_state_broadcaster')
    post_jsb = [spawner('diff_drive_controller')]

    if arm:
        # Arm controller params are already on the controller_manager (the
        # gz_ros2_control plugin loaded controllers.sim.yaml at spawn), so
        # these only need spawning by name.
        suffix = '_trajectory_controller' if arm_control == 'moveit' else '_controller'
        post_jsb.append(spawner('arm' + suffix))
        if arm_tool != 'none':
            post_jsb.append(spawner('gripper' + suffix))

    # Autonomy + manipulation tail, once the controllers are up and odometry
    # is flowing. Each is an independently launchable stack; this composes them.
    tail = []
    if slam:
        tail.append(_stack('base101_slam', 'slam.launch.py'))
    if nav:
        tail.append(_stack('base101_nav', 'nav.launch.py', rviz=rviz))
    if moveit:
        tail.append(_stack('base101_arm_moveit_config', 'move_group.launch.py',
                           arm_tool=arm_tool, rviz=rviz))
    if agent:
        # The robocore agent: JSON-RPC bridge that drives this robot. It is a
        # consumer of everything above (reads /base_camera/*, /scan, /map,
        # commands /cmd_vel_agent at twist_mux priority 50) and it supervises
        # the stacks rather than the reverse — which is why slam runs with
        # bond_timeout 0.0 and the BT has no recovery nodes.
        #
        # Started last, after nav/slam, because it resolves topics and frames
        # at startup against whatever is up.
        tail.append(Node(
            package='robocore_agent',
            executable='agent',
            name='robocore_agent',
            output='screen',
            arguments=['--profile', resolve_profile(arg('profile'), arm),
                       '--port', arg('agent_port'),
                       '--socket', arg('agent_socket')],
            parameters=[{'use_sim_time': True}],
        ))
    if tail:
        post_jsb.append(TimerAction(period=5.0, actions=tail))

    # Controllers must wait until the robot is spawned (which is when the
    # gz_ros2_control plugin creates the controller_manager).
    after_spawn = RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot, on_exit=[spawn_jsb]))
    after_jsb = RegisterEventHandler(OnProcessExit(
        target_action=spawn_jsb, on_exit=post_jsb))

    actions = [
        gz_resource_path,
        robot_state_publisher,
        gz_sim,
        clock_bridge,
        bridge,
        *image_bridges,
        twist_mux,
        spawn_robot,
        after_spawn,
        after_jsb,
        rosboard,
    ]

    if arm:
        actions.append(Node(
            package='ros_gz_image', executable='image_bridge',
            name='arm_wrist_camera_image_bridge',
            arguments=['/arm_wrist_camera/image_raw'],
            parameters=[{'use_sim_time': True}], output='screen'))
        # Intrinsics. ros_gz_image bridges the image stream only, so without
        # this the wrist camera publishes pixels with no camera_info and
        # nothing can project them into 3D — robocore_agent reports it as
        # "camera has no intrinsics". Its own node rather than an entry in
        # base101_worlds' gz_ros_bridge.yaml, because that config is shared
        # with the armless configuration, which has no wrist camera.
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='arm_wrist_camera_info_bridge',
            arguments=['/arm_wrist_camera/camera_info@sensor_msgs/msg/'
                       'CameraInfo[gz.msgs.CameraInfo'],
            parameters=[{'use_sim_time': True}], output='screen'))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arm', default_value='false', choices=['true', 'false'],
            description='Mount one mod101 arm on the deck (needs the mod101 '
                        'underlay sourced).'),
        DeclareLaunchArgument(
            'arm_tool', default_value=_configured_tool(),
            description='mod101 end-effector (mod101_tool_<name>). Defaults to '
                        "the web configurator's saved tool."),
        DeclareLaunchArgument(
            'arm_control', default_value='sliders',
            choices=['sliders', 'moveit'],
            description='Arm controllers to spawn: sliders = position '
                        '(Float64MultiArray, what the web UIs drive), '
                        'moveit = trajectory (FollowJointTrajectory). '
                        'moveit:=true forces this to moveit.'),
        DeclareLaunchArgument(
            'moveit', default_value='false', choices=['true', 'false'],
            description='Start move_group too. Implies arm_control:=moveit; '
                        'requires arm:=true.'),
        DeclareLaunchArgument(
            'nav', default_value='true', choices=['true', 'false'],
            description='Nav2 (planner, controller, bt_navigator, smoother).'),
        DeclareLaunchArgument(
            'slam', default_value='true', choices=['true', 'false'],
            description='EKF + slam_toolbox. Nav2 needs the map frame this '
                        'publishes; with slam:=false nav sits in Activating '
                        'until something else provides it.'),
        DeclareLaunchArgument(
            'agent', default_value='true', choices=['true', 'false'],
            description='Run the robocore agent (JSON-RPC bridge) against '
                        'this robot.'),
        DeclareLaunchArgument(
            'profile', default_value='',
            description='robocore profile YAML. Empty = $ROBOCORE_PROFILE, '
                        'else base101{_arm}.yaml from the engine profiles '
                        'dir, picked by the arm argument.'),
        DeclareLaunchArgument(
            'agent_port', default_value='10101',
            description='Agent TCP port (0 disables). Must match '
                        'robocore.uri.DEFAULT_PORT.'),
        DeclareLaunchArgument(
            'agent_socket', default_value='/tmp/robocore.sock',
            description="Agent unix socket path ('none' disables)."),
        DeclareLaunchArgument(
            'rviz', default_value='false', choices=['true', 'false'],
            description="RViz for whichever stacks are up (nav's display "
                        "config, move_group's). Off by default — rosboard is "
                        'the usual dashboard here.'),
        DeclareLaunchArgument(
            'rosboard', default_value='true', choices=['true', 'false'],
            description='Run the rosboard web dashboard + teleop card.'),
        DeclareLaunchArgument(
            'rosboard_port', default_value='8888',
            description='HTTP/WS port for rosboard.'),
        DeclareLaunchArgument(
            'world', default_value='sticky_floor.sdf',
            description='SDF world (name in base101_worlds/worlds or an '
                        'absolute path).'),
        DeclareLaunchArgument(
            'camera', default_value='realsense', choices=['realsense', 'oak_d'],
            description='Depth module on the front bracket. Only changes the '
                        'mesh and the simulated FOV; topics stay /base_camera/*.'),
        OpaqueFunction(function=_setup),
    ])
