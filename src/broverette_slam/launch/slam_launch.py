from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Path to the slam_toolbox launch file
    slam_toolbox_launch_dir = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')

    params_file = os.path.join(
        get_package_share_directory('broverette_slam'), 'config', 'params.yaml'
    )

    # Include the SLAM Toolbox's online async launch file
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_dir),
        launch_arguments={
            # 'base_frame': 'base_link',
            # 'odom_frame': 'odom',
            'slam_params_file': params_file,
            'use_sim_time': 'false',  # Adjust based on your needs
        }.items(),
    )

    urdf_file = os.path.join(
        get_package_share_directory('broverette_description'),
        'urdf',
        'broverette.urdf'
    )

    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
            }
        ]
    )

    # Add the path to your existing broverette RViz config file from broverette_description
    rviz_config_file = os.path.join(
        get_package_share_directory('broverette_description'), 'rviz', 'broverette.rviz')

    # Node to launch RViz with the broverette.rviz configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        slam_toolbox_launch,
        robot_state_publisher_node,
        rviz_node,
    ])
