from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roar_transforms',
            executable='roar_tf2_broadcaster',
            name='roar_tf2_broadcaster',
        ),
    ])