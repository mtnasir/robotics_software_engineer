#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the TurtleBot3 Gazebo launch file
    tb3_gazebo_launch_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    tb3_gazebo_launch_file = os.path.join(tb3_gazebo_launch_dir, 'empty_world.launch.py')
    rviz_config_file = os.path.join(
           get_package_share_directory('rse_m5'),
           'rviz',
           '2.rviz'
       )
    # Node for the goal planner
    goal_planner_node = Node(
        package='rse_m5',
        executable='non',
        name='multi_goal_follow'
    )
    rr=Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d',rviz_config_file]
        )
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb3_gazebo_launch_file)
        ),
        goal_planner_node,
        rr
    ])
