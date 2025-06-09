import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generates the launch description for starting the param_talker node
    with parameters loaded from a YAML file.
    """
    # 1. Define the path to the package.
    pkg_path = get_package_share_directory('py_pubsub')

    # 2. Define the path to the parameter file.
    param_file_path = os.path.join(pkg_path, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='py_pubsub',
            executable='param_talker',
            name='minimal_publisher',  # Must match the node name in the YAML file
            # 3. Pass the parameter file to the node.
            parameters=[param_file_path]
        )
    ])
