import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    share_dir = get_package_share_directory('base101_description')
    variant = LaunchConfiguration('variant').perform(context)
    tower = LaunchConfiguration('tower').perform(context)
    arms = LaunchConfiguration('arms').perform(context)
    arm_tool = LaunchConfiguration('arm_tool').perform(context)

    xacro_file = os.path.join(share_dir, 'urdf', 'base101.xacro')
    # display is rviz-only — skip the per-sim ros2_control + extensions so
    # robot_state_publisher sees a pure URDF.
    robot_urdf = xacro.process_file(
        xacro_file, mappings={'variant': variant, 'simulator': 'none',
                              'tower': tower, 'arms': arms,
                              'arm_tool': arm_tool}
    ).toxml()

    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')
    show_gui = LaunchConfiguration('gui')

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_urdf}],
        ),
        Node(
            condition=UnlessCondition(show_gui),
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
        ),
        Node(
            condition=IfCondition(show_gui),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'variant',
            default_value='simple',
            choices=['simple', 'pro'],
            description='Which base101 hardware variant to load (simple or pro).',
        ),
        DeclareLaunchArgument(
            'tower',
            default_value='false',
            choices=['true', 'false'],
            description='Include the cross tower (lift column + pan/tilt head).',
        ),
        DeclareLaunchArgument(
            'arms',
            default_value='false',
            choices=['true', 'false'],
            description='Mount two mod101 arms on the tower crossbeam '
                        '(requires tower:=true and the mod101 underlay).',
        ),
        DeclareLaunchArgument(
            'arm_tool',
            default_value='jaws',
            description='mod101 end-effector for both arms.',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='True',
            description='Use joint_state_publisher_gui instead of joint_state_publisher.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
