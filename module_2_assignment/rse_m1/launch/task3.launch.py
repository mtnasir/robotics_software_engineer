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
             "\"{x: 4.5, y: 4.5, theta: 1.57, name: 'turtle2'}\""],
        name='spawn_turtle2',
        shell=True
    ) 
    spawm_t3=ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', 
             "\"{x: 3.5, y: 3.5, theta: 1.57, name: 'turtle3'}\""],
        name='spawn_turtle3',
        shell=True
    ) 
    spawm_t4=ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', 
             "\"{x: 6.5, y: 6.5, theta: 1.57, name: 'turtle4'}\""],
        name='spawn_turtle4',
        shell=True
    ) 
    spawm_t5=ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', 
             "\"{x: 7.5, y: 7.5, theta: 1.57, name: 'turtle5'}\""],
        name='spawn_turtle5',
        shell=True
    ) 
    turtle_drive=Node(
        package='rse_m1',
        executable='turtle_drive',
        name='rse1'
    )
    turtle_drive2=Node(
        package='rse_m1',
        executable='turtle_drive',
        name='rse2',
        parameters=[
            {'t_cmd_vel':'/turtle2/cmd_vel',
            't_cmd_fun' : 'b' }
        ]
    )
    turtle_drive3=Node(
        package='rse_m1',
        executable='turtle_drive',
        name='rse3',
        parameters=[
            {'t_cmd_vel':'/turtle4/cmd_vel',
            't_cmd_fun' : 'c' }
        ]
    )
    clear_t=ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/clear std_srvs/srv/Empty '],
        name='clear_turtule',
        shell=True
    )
    return LaunchDescription([
        turtlesim,
        spawm_t2,
        spawm_t3,
        spawm_t4,
        spawm_t5,
        turtle_drive,
        turtle_drive2,
        turtle_drive3,
        clear_t
    ])