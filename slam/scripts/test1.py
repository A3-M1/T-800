#!/usr/bin/python3

import subprocess
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="Lancement de slam_toolbox avec use_sim_time:=True"),
        
        # Exécution de la commande via subprocess
        LogInfo(
            condition=None,
            msg="Lancement de la commande ros2 launch slam_toolbox"
        ),
        
        Node(
            package='slam_toolbox',
            executable='online_sync_launch.py',
            name='slam_toolbox',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])


