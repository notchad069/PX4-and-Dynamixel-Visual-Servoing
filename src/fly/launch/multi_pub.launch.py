from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fly',
            namespace='offboard',
            executable='mode_node',
            name='mode_node'
        ),
        Node(
            package='fly',
            namespace='vehiclecmd',
            executable='cmd_node',
            name='cmd_node'
        ),
        Node(
            package='fly',
            namespace='trajectorysp',
            executable='pos_node',
            name='pos_node'
        ),
    ])

