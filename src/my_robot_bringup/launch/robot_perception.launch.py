"""Independent Vidu capture, perception and optional saved hand-eye TF."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def boolean(name):
        text = value(name).lower()
        if text not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        return text == 'true'

    mode = value('perception_mode')
    if mode not in ('color', 'aruco', 'both', 'none'):
        raise ValueError('perception_mode must be color, aruco, both or none')
    config = value('perception_config')
    if not Path(config).is_file():
        raise ValueError(f'Configuration does not exist: {config}')
    common = {'use_sim_time': boolean('use_sim_time')}
    show_window = boolean('show_window')
    nodes = []

    def add(executable, package='my_robot_perception', **parameters):
        nodes.append(Node(package=package, executable=executable,
                          parameters=[config, {**common, **parameters}], output='screen'))

    if boolean('start_camera'):
        add('vidu_ros2_wrapper_node', 'vidu_ros2_wrapper')
    if mode in ('color', 'both'):
        enable_depth = boolean('enable_depth')
        parameters = {'show_window': show_window, 'enable_detection': True}
        if not enable_depth:
            parameters['depth_points_topic'] = ''
        add('color_detector', **parameters)
        if enable_depth:
            add('depth_localizer')
    if mode in ('aruco', 'both'):
        add('aruco_detector', show_window=show_window)
    if mode == 'none' and show_window:
        add('color_detector', show_window=True, enable_detection=False)

    if boolean('publish_handeye_tf'):
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
        DeclareLaunchArgument('publish_handeye_tf', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('handeye_name', default_value=''),
        DeclareLaunchArgument('handeye_config', default_value=''),
        DeclareLaunchArgument('handeye_directory', default_value='calibration/handeye'),
        OpaqueFunction(function=make_nodes),
    ])
