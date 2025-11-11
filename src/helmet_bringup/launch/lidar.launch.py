#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch RoboSense LiDAR node"""
    
    # Declare launch arguments
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='lidar_link',
        description='Frame ID for LiDAR data'
    )
    
    # Get launch configurations
    frame_id = LaunchConfiguration('frame_id')
    
    # Get package directories
    helmet_bringup_dir = FindPackageShare('helmet_bringup')
    
    # LiDAR node
    lidar_node = Node(
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        name='lidar_node',
        parameters=[
            PathJoinSubstitution([
                helmet_bringup_dir,
                'config',
                'lidar_params.yaml'
            ]),
            {
                'frame_id': frame_id,
            }
        ],
        output='screen'
    )
    
    return LaunchDescription([
        frame_id_arg,
        lidar_node,
    ])