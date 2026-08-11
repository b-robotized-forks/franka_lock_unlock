import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('franka_lock_unlock'),
        'config',
        'franka_lock_unlock_params.yaml'
    )

    node = Node(
        package = 'franka_lock_unlock',
        name  = 'franka_lock_unlock_node',
        executable = 'franka_lock_unlock_node',
        parameters=[config],
        ros_arguments=['--log-level', 'franka_lock_unlock_node:=debug']
    )

    ld.add_action(node)
    return ld
