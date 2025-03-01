# sensor_fusion_launch.py
import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Path to your EKF parameter file
    ekf_params_file = os.path.join(
        get_package_share_directory('sensor_fusion'),
        'config',
        'ekf_params.yaml'
    )
    rviz_config_path = os.path.join(get_package_share_directory('sensor_fusion'), 'config', 'sensor_fusion.rviz')

    # Include TurtleBot3 Gazebo launch file
    # Adjust 'turtlebot3_gazebo' and 'turtlebot3_world.launch.py' to your setup
    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'empty_world.launch.py'
            )
        ]),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )
    rvizn=Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    # EKF node from robot_localization
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file],
        remappings=[
            ('/odometry/filtered', '/odom/filtered'),
            ('/imu/data', '/imu/data'),
            ('/gps/fix', '/gps/fix')
        ]
    )

    return launch.LaunchDescription([
        turtlebot3_gazebo,
        ekf_node,
        rvizn
    ])