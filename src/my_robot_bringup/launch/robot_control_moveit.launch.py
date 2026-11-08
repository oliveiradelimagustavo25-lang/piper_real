"""Independent Piper + AGX gripper control stack; no task or vision nodes."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def make_nodes(context):
    real = LaunchConfiguration('control_mode').perform(context) == 'real'
    package, entry = ('agx_arm_ctrl', 'start_single_agx_arm_moveit.launch.py') if real else (
        'agx_arm_moveit', 'demo.launch.py')
    arguments = {
        'arm_type': 'piper', 'effector_type': 'agx_gripper',
        'follow': str(real).lower(), 'auto_control_gate': str(real).lower(),
        'use_rviz': LaunchConfiguration('use_rviz'),
        'tcp_offset': LaunchConfiguration('tcp_offset'),
    }
    if real:
        arguments.update(can_port=LaunchConfiguration('can_port'),
                         auto_enable=LaunchConfiguration('auto_enable'))
    return [GroupAction([
        SetParameter('use_sim_time', LaunchConfiguration('use_sim_time')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                Path(get_package_share_directory(package)) / 'launch' / entry)),
            launch_arguments=arguments.items()),
    ])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('control_mode', default_value='mock', choices=['mock', 'real']),
        DeclareLaunchArgument('can_port', default_value='can0'),
        DeclareLaunchArgument('auto_enable', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('tcp_offset', default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=make_nodes),
    ])
