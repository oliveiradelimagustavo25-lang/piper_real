"""Visual pipeline only; opening the camera is an explicit opt-in."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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

    mode = value('mode')
    if mode not in ('color', 'aruco', 'both', 'none'):
        raise ValueError('mode must be color, aruco, both or none')
    config = value('config')
    if not Path(config).is_file():
        raise ValueError(f'Configuration does not exist: {config}')
    common = {'use_sim_time': boolean('use_sim_time')}
    show_window = boolean('show_window')
    nodes = []

    def add(executable, package='my_robot_perception', **parameters):
        overrides = dict(common)
        overrides.update(parameters)
        nodes.append(Node(package=package, executable=executable,
                          parameters=[config, overrides], output='screen'))

    if boolean('start_camera'):
        add('vidu_ros2_wrapper_node', 'vidu_ros2_wrapper')
    if mode in ('color', 'both'):
        enable_depth = boolean('enable_depth')
        color_parameters = dict(show_window=show_window, enable_detection=True)
        if not enable_depth:
            color_parameters['depth_points_topic'] = ''
        add('color_detector', **color_parameters)
        if enable_depth:
            add('depth_localizer')
    if mode in ('aruco', 'both'):
        add('aruco_detector', show_window=show_window)
    if mode == 'none' and show_window:
        add('color_detector', show_window=True, enable_detection=False)
    return nodes


def generate_launch_description():
    default_config = str(Path(get_package_share_directory('my_robot_perception')) /
                         'config' / 'perception.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config,
                              description='Single YAML configuration for all visual nodes'),
        DeclareLaunchArgument('start_camera', default_value='false', choices=['true', 'false'],
                              description='Open the physical Vidu camera'),
        DeclareLaunchArgument('mode', default_value='color',
                              choices=['color', 'aruco', 'both', 'none'],
                              description='Select detection and windows; none previews RGB only'),
        DeclareLaunchArgument('enable_depth', default_value='true', choices=['true', 'false'],
                              description='Run depth localization in color/both mode'),
        DeclareLaunchArgument('show_window', default_value='true', choices=['true', 'false'],
                              description='Master GUI switch; false disables both windows'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=make_nodes),
    ])
