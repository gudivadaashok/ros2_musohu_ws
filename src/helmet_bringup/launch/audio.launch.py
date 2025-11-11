#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch only the ReSpeaker processing node"""
    
    # Get package directories
    helmet_bringup_dir = FindPackageShare('helmet_bringup')
    
    # ReSpeaker processing node
    respeaker_node = Node(
        package='respeaker_ros',
        executable='respeaker_node',
        name='respeaker_node',
        parameters=[
            PathJoinSubstitution([
                helmet_bringup_dir,
                'config',
                'audio_params.yaml'
            ])
        ],
        output='screen'
    )
    
    return LaunchDescription([
        respeaker_node,
    ])