"""Task planning and grasp execution, consuming the control stack's model and state."""
from pathlib import Path
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def make_nodes(context):
    real = LaunchConfiguration('control_mode').perform(context) == 'real'
    common = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    nodes = []
    if all(LaunchConfiguration(key).perform(context) == 'true'
           for key in ('start_executor', 'start_planner')):
        def parameters(key, node):
            path = Path(LaunchConfiguration(key).perform(context))
            return yaml.safe_load(path.read_text())[node]['ros__parameters']

        executor = parameters('manipulation_config', 'pick_place_executor')
        planner = parameters('planner_config', 'task_planner')
        for first, second, default in (
                ('reference_frame', 'reference_frame', 'base_link'),
                ('end_effector_link', 'end_effector_frame', 'link6'),
                ('tool_to_tcp', 'tool_to_tcp', [0., 0., .132])):
            value = executor.get(first, default)
            if value != planner.get(second, default):
                raise ValueError(f'Executor {first} and planner {second} must agree')
            if first == 'tool_to_tcp' and (not isinstance(value, list) or len(value) != 3 or any(
                    isinstance(v, bool) or not isinstance(v, (int, float)) or
                    not math.isfinite(v) for v in value)):
                raise ValueError('tool_to_tcp must contain three finite numbers in metres')
    if LaunchConfiguration('start_executor').perform(context) == 'true':
        kinematics = Path(get_package_share_directory('agx_arm_moveit')) / 'config/kinematics.yaml'
        nodes.append(Node(
            package='my_robot_commander', executable='pick_place_executor', output='screen',
            parameters=[LaunchConfiguration('manipulation_config'), common,
                        {'robot_description_kinematics': yaml.safe_load(kinematics.read_text())}],
            remappings=[('joint_states', '/feedback/joint_states' if real else
                         '/control/joint_states')]))
    if LaunchConfiguration('start_planner').perform(context) == 'true':
        nodes.append(Node(
            package='my_robot_planner', executable='task_planner', output='screen',
            parameters=[LaunchConfiguration('planner_config'), common]))
    return nodes


def generate_launch_description():
    def config(package, filename):
        return str(Path(get_package_share_directory(package)) / 'config' / filename)

    return LaunchDescription([
        DeclareLaunchArgument('control_mode', default_value='real', choices=['mock', 'real']),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('start_executor', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_planner', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('manipulation_config',
                              default_value=config('my_robot_commander', 'manipulation.yaml')),
        DeclareLaunchArgument('planner_config',
                              default_value=config('my_robot_planner', 'task_planner.yaml')),
        OpaqueFunction(function=make_nodes),
    ])
