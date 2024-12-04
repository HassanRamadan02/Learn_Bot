import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'learn_one'  # Change this to your package name

    # Include the robot_state_publisher launch file, provided by your package
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # Twist Mux configuration
    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    # Gazebo world setup
    default_world = os.path.join(get_package_share_directory(package_name), 'worlds', 'empty.world')
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]), launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Spawn the robot entity in Gazebo
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description', '-name', 'my_bot', '-z', '0.1'],
                        output='screen')

    # Spawner nodes for controllers
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],

    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],

    )

    # Bridge and image bridge configurations
    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}']
    )

     # Include the controller manager 
    controller_manager = Node( 
        package='controller_manager', 
        executable='ros2_control_node',
        parameters=[os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml')],
        output='screen'
    ) 

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    return LaunchDescription([
        rsp,
        controller_manager,
        twist_mux,
        world_arg,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        ros_gz_bridge,
        ros_gz_image_bridge
    ])
