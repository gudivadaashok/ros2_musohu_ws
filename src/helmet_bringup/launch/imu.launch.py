#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch Witmotion IMU node"""
    
    # Declare launch arguments
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/ttyUSB0',
        description='Serial device for IMU'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='Frame ID for IMU data'
    )
    
    # Get launch configurations
    device = LaunchConfiguration('device')
    frame_id = LaunchConfiguration('frame_id')
    
    # Get package directories
    helmet_bringup_dir = FindPackageShare('helmet_bringup')
    
    # IMU node
    imu_node = Node(
        package='witmotion_ros2',
        executable='witmotion_ros2',
        name='imu_node',
        parameters=[
            PathJoinSubstitution([
                helmet_bringup_dir,
                'config',
                'imu_params.yaml'
            ]),
            {
                'device': device,
                'frame_id': frame_id,
            }
        ],
        output='screen'
    )
    
    return LaunchDescription([
        device_arg,
        frame_id_arg,
        imu_node,
    ])
