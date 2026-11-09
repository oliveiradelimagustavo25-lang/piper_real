"""Independent Vidu capture and perception; reuses the module's visual pipeline."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def make_nodes(context):
    share = Path(get_package_share_directory('my_robot_perception'))
    nodes = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch/perception_test.launch.py')),
        launch_arguments={
            'config': LaunchConfiguration('perception_config'),
            'mode': LaunchConfiguration('perception_mode'),
            **{name: LaunchConfiguration(name) for name in (
                'start_camera', 'enable_depth', 'show_window', 'use_sim_time')},
        }.items())]
    if LaunchConfiguration('publish_handeye_tf').perform(context) == 'true':
        bringup = Path(get_package_share_directory('my_robot_bringup'))
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch/robot_handeye.launch.py')),
            launch_arguments={
                'mode': 'publish',
                'name': LaunchConfiguration('handeye_name'),
                'handeye_config': LaunchConfiguration('handeye_config'),
                'data_directory': LaunchConfiguration('handeye_directory'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items()))
    return nodes


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
        DeclareLaunchArgument('publish_handeye_tf', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('handeye_name', default_value=''),
        DeclareLaunchArgument('handeye_config', default_value=''),
        DeclareLaunchArgument('handeye_directory', default_value='calibration/handeye'),
        OpaqueFunction(function=make_nodes),
    ])
