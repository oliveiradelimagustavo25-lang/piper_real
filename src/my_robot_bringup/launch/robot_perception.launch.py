"""Independent Vidu capture and perception; reuses the module's visual pipeline."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def make_nodes(context):
    share = Path(get_package_share_directory('my_robot_perception'))
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch/perception_test.launch.py')),
        launch_arguments={
            'config': LaunchConfiguration('perception_config'),
            'mode': LaunchConfiguration('perception_mode'),
            **{name: LaunchConfiguration(name) for name in (
                'start_camera', 'enable_depth', 'show_window', 'use_sim_time')},
        }.items())]


def generate_launch_description():
    share = Path(get_package_share_directory('my_robot_perception'))
    return LaunchDescription([
        DeclareLaunchArgument('perception_config',
                              default_value=str(share / 'config/perception.yaml')),
        DeclareLaunchArgument('perception_mode', default_value='color',
                              choices=['color', 'aruco', 'both', 'none']),
        DeclareLaunchArgument('start_camera', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_depth', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('show_window', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=make_nodes),
    ])
