from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for the publisher-subscriber system.
    This function is the entry point for a ROS 2 launch file.
    """
    return LaunchDescription([
        Node(
            package='py_pubsub',
            executable='talker',
            name='my_talker_node'  # Optional: Assign a custom name to the node
        ),
        Node(
            package='py_pubsub',
            executable='listener',
            name='my_listener_node' # Optional: Assign a custom name
        )
    ])