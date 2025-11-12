# ROS2 Musohu Workspace

This is a ROS2 workspace for the Musohu robotic helmet project running on Jetson hardware.

## Overview

This workspace contains packages for:
- **ZED Camera**: Stereoscopic vision and depth sensing
- **Audio Processing**: ReSpeaker microphone array and audio capture
- **LiDAR**: Laser range finding sensors  
- **IMU**: Inertial measurement unit for orientation tracking
- **Helmet Integration**: Launch files and configuration for the complete sensor suite

## Hardware

- **Platform**: NVIDIA Jetson
- **Camera**: ZED 2i Stereoscopic Camera
- **Audio**: ReSpeaker Microphone Array
- **LiDAR**: RS LiDAR sensors
- **IMU**: Witmotion IMU sensors

## Quick Start

1. **Build the workspace**:
   ```bash
   cd /home/jetson/ros2_musohu_ws
   colcon build
   source install/setup.bash
   ```

2. **Launch individual sensors**:
   ```bash
   # ZED Camera
   ros2 launch helmet_bringup zed.launch.py
   
   # Audio capture
   ros2 launch helmet_bringup audio.launch.py
   
   # LiDAR
   ros2 launch helmet_bringup lidar.launch.py
   
   # IMU
   ros2 launch helmet_bringup imu.launch.py
   ```

3. **Launch complete sensor suite**:
   ```bash
   ros2 launch helmet_bringup helmet_sensors.launch.py
   ```

## Configuration

Key configuration files:
- `src/helmet_bringup/config/zed_params.yaml` - ZED camera parameters (optimized for reduced frame corruption)
- Other sensor configurations in respective package config directories

## Recent Updates

- **ZED Camera Optimization**: Reduced frame rate from 30Hz to 15Hz and added frame validation to eliminate corrupted frame warnings
- **Stable Configuration**: All sensors configured for reliable operation on Jetson hardware

## Packages Included

- `helmet_bringup` - Main integration and launch files
- `zed-ros2-wrapper` - ZED camera ROS2 wrapper
- `audio_common` - Audio processing utilities
- `respeaker_ros` - ReSpeaker microphone array driver
- `rslidar_msg` & `rslidar_sdk` - LiDAR sensor drivers
- `witmotion_ros2` - IMU sensor driver

## Development

Built and tested on:
- **ROS2**: Humble
- **OS**: Ubuntu (Jetson Linux)
- **Date**: November 2025


## Active ROS2 Topics

Current active topics when all sensors are running:

### **ZED Camera Topics**
- `/zed2i/zed_node/rgb/color/rect/image/compressed` - RGB camera stream
- `/zed2i/zed_node/depth/depth_registered/compressed` - Depth camera stream  
- `/zed2i/zed_node/imu/data` - IMU sensor data
- `/zed2i/zed_node/odom` - Visual odometry
- `/zed2i/zed_node/pose` - Camera pose estimation

### **Audio Processing Topics**
- `/audio` - Main audio stream
- `/audio/channel0-5` - Individual microphone channels
- `/doa` - Direction of arrival estimation
- `/speech_audio` - Speech detection
- `/vad` - Voice activity detection

### **LiDAR Topics**
- `/rslidar_points` - Point cloud data from RS LiDAR

### **IMU Top[]
- `/imu_node/cali`

### **System Topics**
- `/tf` & `/tf_static` - Transform data
- `/diagnostics` - System diagnostics
- `/rosout` - ROS logging

Use `ros2 topic list` to see all currently active topics.
