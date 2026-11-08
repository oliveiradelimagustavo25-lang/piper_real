"""Compose the three independent layers and expose their launch arguments."""
from pathlib import Path

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declarations, layers, declared = [], [], set()
    for entry in ('robot_control_moveit.launch.py',
                  'robot_motion_planning.launch.py', 'robot_perception.launch.py'):
        source = PythonLaunchDescriptionSource(str(Path(__file__).with_name(entry)))
        description = source.get_launch_description(LaunchContext())
        arguments = {}
        for action in description.entities:
            if isinstance(action, DeclareLaunchArgument):
                if action.name not in declared:
                    declarations.append(action)
                    declared.add(action.name)
                arguments[action.name] = LaunchConfiguration(action.name)
        layers.append(GroupAction([
            IncludeLaunchDescription(source, launch_arguments=arguments.items()),
        ]))
    return LaunchDescription([*declarations, *layers])
