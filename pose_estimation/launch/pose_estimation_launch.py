from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mediapipe_pose',
            executable='mediapipe_pose_node',
            name='mediapipe_pose_node',
            output='screen',
        )
    ])
