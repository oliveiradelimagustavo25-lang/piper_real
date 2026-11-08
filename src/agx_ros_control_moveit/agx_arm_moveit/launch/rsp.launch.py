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
            package="robot_state_publisher",
            executable="robot_state_publisher",
            respawn=True,
            output="screen",
            parameters=[moveit_config.robot_description],
            remappings=[("joint_states", joint_states_topic)],
        )
    ]


def generate_launch_description():
    return LaunchDescription(declare_common_args() + [OpaqueFunction(function=_launch)])
