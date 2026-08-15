import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('base101_simple_description')
    rviz_dir = get_package_share_directory('base101_description')

    xacro_file = os.path.join(pkg, 'urdf', 'base101_simple.xacro')
    # display is rviz-only — skip the per-sim ros2_control + extensions so
    # robot_state_publisher sees a pure URDF. The arm/tower variants have their
    # own display launches.
    robot_urdf = xacro.process_file(
        xacro_file, mappings={
            'simulator': 'none',
            'camera': LaunchConfiguration('camera').perform(context),
        }
    ).toxml()

    rviz_config_file = os.path.join(rviz_dir, 'config', 'display.rviz')
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
            'gui',
            default_value='True',
            description='Use joint_state_publisher_gui instead of joint_state_publisher.',
        ),
        DeclareLaunchArgument(
            'camera',
            default_value='realsense',
            choices=['realsense', 'oak_d'],
            description='Depth module on the front bracket.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
