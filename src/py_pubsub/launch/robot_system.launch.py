import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for the complete robot system.
    """
    pkg_path = get_package_share_directory('py_pubsub')
    param_file_path = os.path.join(pkg_path, 'config', 'robot_system_params.yaml')

    # Node for the robot_controller
    robot_controller_node = Node(
        package='py_pubsub',
        executable='robot_controller',
        name='robot_controller',
        output='screen',
        parameters=[param_file_path]
    )

    # Node for the robot_commander
    robot_commander_node = Node(
        package='py_pubsub',
        executable='robot_commander',
        name='robot_commander',
        output='screen'
    )

    return LaunchDescription([
        robot_controller_node,
        robot_commander_node
    ])
