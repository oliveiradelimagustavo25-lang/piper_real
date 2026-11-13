"""Calibrate from existing TF streams, or publish a saved hand-eye transform."""
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
        return value(name) == 'true'

    share = Path(get_package_share_directory('my_robot_bringup'))
    parameters = [{
        'name': 'piper_vidu', 'calibration_type': 'eye_in_hand',
        'robot_base_frame': 'base_link', 'robot_effector_frame': 'link6',
        'tracking_base_frame': 'camera_color_optical_frame',
        'tracking_marker_frame': 'aruco_marker_9',
    }]
    if value('handeye_config'):
        config = Path(value('handeye_config')).expanduser().resolve()
        if not config.is_file():
            raise ValueError(f'Hand-eye parameter file does not exist: {config}')
        parameters.append(str(config))
    overrides = {name: value(name) for name in (
        'name', 'robot_base_frame', 'robot_effector_frame',
        'tracking_base_frame', 'tracking_marker_frame') if value(name)}
    directory = str(Path(value('data_directory')).expanduser().resolve())
    parameters.append({**overrides, 'use_sim_time': boolean('use_sim_time'),
                       'data_directory': directory})
    if value('mode') == 'publish':
        return [Node(package='easy_handeye2', executable='handeye_publisher',
                     name='handeye_publisher', parameters=parameters, output='screen')]

    nodes = []
    if boolean('start_control'):
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/robot_control_moveit.launch.py')),
            launch_arguments={
                'control_mode': 'real',
                **{name: LaunchConfiguration(name) for name in (
                    'can_port', 'auto_enable', 'use_rviz', 'use_sim_time')},
            }.items()))
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch/robot_perception.launch.py')),
        launch_arguments={
            'perception_mode': 'aruco', 'enable_depth': 'false',
            # Sampling must not load the previous calibration through perception defaults.
            'publish_handeye_tf': 'false',
            **{name: LaunchConfiguration(name) for name in (
                'perception_config', 'start_camera', 'show_window', 'use_sim_time')},
        }.items()))
    nodes.append(Node(package='easy_handeye2', executable='handeye_server',
                      name='handeye_server', parameters=parameters, output='screen'))
    if boolean('show_gui'):
        nodes.append(Node(package='easy_handeye2', executable='rqt_calibrator.py',
                          name='handeye_rqt_calibrator', parameters=parameters, output='screen'))
    return nodes


def generate_launch_description():
    perception = Path(get_package_share_directory('my_robot_perception'))
    arguments = [
        DeclareLaunchArgument('mode', default_value='calibrate', choices=['calibrate', 'publish']),
        DeclareLaunchArgument('handeye_config', default_value=''),
        DeclareLaunchArgument('perception_config',
                              default_value=str(perception / 'config/perception.yaml')),
        DeclareLaunchArgument('data_directory', default_value='calibration/handeye',
                              description='Result and sample directory; relative to launch cwd'),
        DeclareLaunchArgument('can_port', default_value='can0'),
    ]
    for name, default in (
        ('start_control', 'false'), ('auto_enable', 'false'), ('use_rviz', 'true'),
        ('start_camera', 'true'), ('show_window', 'true'), ('show_gui', 'true'),
        ('use_sim_time', 'false'),
    ):
        arguments.append(DeclareLaunchArgument(name, default_value=default,
                                               choices=['true', 'false']))
    for name in ('name', 'robot_base_frame', 'robot_effector_frame',
                 'tracking_base_frame', 'tracking_marker_frame'):
        arguments.append(DeclareLaunchArgument(
            name, default_value='', description='Optional override of node defaults or YAML'))
    return LaunchDescription([*arguments, OpaqueFunction(function=make_nodes)])
