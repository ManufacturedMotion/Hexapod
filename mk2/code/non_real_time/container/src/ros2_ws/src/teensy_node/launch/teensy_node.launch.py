from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    teensy_node = Node(
        package="teensy_node",
        executable="teensy_node",
    )
    ld.add_action(teensy_node)
    return ld