import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from _moveit_config_builder import build_moveit_config, declare_common_args, joint_state_topic


def _launch(context):
    joint_states_topic = joint_state_topic(context)
    moveit_config = build_moveit_config(context)
    return [
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
            remappings=[("joint_states", joint_states_topic)],
        )
    ]


def generate_launch_description():
    return LaunchDescription(declare_common_args() + [OpaqueFunction(function=_launch)])
