#!/usr/bin/env python3
"""
rover_run.launch.py: main launch file of the rover

Calls joy.launch.py and opens 4 terminals to run the commands
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Including joy.launch.py
    joy_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('control2'),   
                'launch',
                'joy.launch.py'                      
            ])
        ])
    )

    commands = [
        'ros2 topic echo /odom',
        'ros2 topic echo /scan',
        'ros2 run turtlesim turtle_teleop_key',
        'htop'
    ]

    terminals = [
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c', f'{cmd}; exec bash'],
            name=f'term_{idx}',
            output='screen'
        )
        for idx, cmd in enumerate(commands, start=1)
    ]

    return LaunchDescription([joy_launch_file, *terminals])