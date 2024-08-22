from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    turtlesim=Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim'
    )
    spawm_t2=ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', 
             "\"{x: 5, y: 5, theta: 1.57, name: 'turtle2'}\""],
        name='spawn_turtle2',
        shell=True
    ) 
    turtle_dirver=Node(
        package='rse_m1',
        executable='rse1',
        name='rse1'
    )
    turtle_dirver2=Node(
        package='rse_m1',
        executable='rse1',
        name='rse2',
        parameters=[
            {'t_cmd_vel':'/turtle2/cmd_vel'}
        ]
    )
    return LaunchDescription([
        turtlesim,
        turtle_dirver,

        spawm_t2,
        turtle_dirver2
    ])