#!/usr/bin/env python3
"""move_group for the composed base101 + mod101 robot.

The counterpart of mod101_moveit_config/launch/move_group.launch.py, but built
from base101_description/urdf/base101.xacro with arm:=true instead of the
standalone arm, so the planning scene contains the chassis the arm is bolted to.

This starts move_group only, and is a stack launch: normally you do not run it
directly, you ask the bringup for it, which starts the sim with the trajectory
controllers and then move_group in the right order —

    ros2 launch base101_bringup_gazebo sim.launch.py arm:=true moveit:=true

Running it by hand against an already-up sim still works:

    ros2 launch base101_bringup_gazebo sim.launch.py arm:=true arm_control:=moveit
    ros2 launch base101_arm_moveit_config move_group.launch.py

Launch args:
    arm_tool                                  mod101_tool_<name>; default is
                                              whatever the configurator saved
    shoulder_ext_length / elbow_ext_length    2020 rail length, m
    shoulder_mount / elbow_mount              small | big
    use_sim_time                              default true
    rviz                                      launch RViz too, default true

The four build args default to empty, meaning "whatever mod101_config.xacro
says" — the configurator owns them and a launch must not shadow them with a
stale number. Pass a value to override.
"""

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

BUILD_ARGS = ('shoulder_ext_length', 'elbow_ext_length',
              'shoulder_mount', 'elbow_mount')


def _configured_tool():
    """The end-effector the mod101 configurator last saved."""
    try:
        cfg = os.path.join(get_package_share_directory('mod101_description'),
                           'urdf', 'mod101_config.xacro')
        m = re.search(r'<xacro:arg\s+name="tool"\s+default="([^"]+)"', open(cfg).read())
        return m.group(1) if m else 'jaws'
    except Exception:
        return 'jaws'


def _drop_unset(mappings):
    """Empty args fall through to mod101_config.xacro's defaults."""
    return {k: v for k, v in mappings.items() if v != ''}


def build_moveit_config(context):
    tool = LaunchConfiguration('arm_tool').perform(context)
    mappings = _drop_unset(
        {k: LaunchConfiguration(k).perform(context) for k in BUILD_ARGS})

    pkg = get_package_share_directory('base101_arm_moveit_config')
    urdf = os.path.join(get_package_share_directory('base101_description'),
                        'urdf', 'base101.xacro')
    srdf = os.path.join(pkg, 'srdf', 'base101_arm.srdf.xacro')

    return (
        MoveItConfigsBuilder('base101', package_name='base101_arm_moveit_config')
        # simulator=gazebo keeps the gz_ros2_control blocks in the description
        # so this URDF matches the one the sim spawned byte for byte.
        .robot_description(file_path=urdf,
                           mappings={**mappings, 'arm': 'true',
                                     'arm_tool': tool, 'simulator': 'gazebo'})
        .robot_description_semantic(file_path=srdf, mappings={'arm_tool': tool})
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        # Unlike mod101 standalone, the gripper entry is not merged in from the
        # tool package: those name the unprefixed joint "6" and here it is arm_6,
        # so base101_arm_control declares both controllers and this file lists
        # both. Nothing tool-specific to fold in.
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .to_moveit_configs()
    )


def _setup(context, *args, **kwargs):
    moveit_config = build_moveit_config(context)
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict(), use_sim_time],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(
            get_package_share_directory('base101_arm_moveit_config'),
            'config', 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            use_sim_time,
        ],
    )
    return [move_group, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm_tool', default_value=_configured_tool()),
        # Empty = "whatever the configurator last saved"; see _drop_unset.
        DeclareLaunchArgument('shoulder_ext_length', default_value=''),
        DeclareLaunchArgument('elbow_ext_length', default_value=''),
        DeclareLaunchArgument('shoulder_mount', default_value=''),
        DeclareLaunchArgument('elbow_mount', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
