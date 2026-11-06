from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('my_robot_planner'), 'config', 'task_planner.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='my_robot_planner', executable='task_planner', output='screen',
             parameters=[LaunchConfiguration('config'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
