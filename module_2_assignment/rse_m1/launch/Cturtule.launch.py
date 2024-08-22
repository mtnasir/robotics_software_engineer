from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    turtlesim=Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim'

    )


    turtle_dirver=Node(
        package='rse_m1',
        executable='rse1',
        name='rse1'

    )


    return LaunchDescription([
        turtlesim,
        turtle_dirver
    ])