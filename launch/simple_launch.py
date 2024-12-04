import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'learn_one'

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml')],
        output='screen'
    )

    return LaunchDescription([
        controller_manager
    ])
