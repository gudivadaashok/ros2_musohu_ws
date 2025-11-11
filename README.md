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

### **IMU Topics**
- `/imu_node/cali` - IMU calibration data

### **System Topics**
- `/tf` & `/tf_static` - Transform data
- `/diagnostics` - System diagnostics
- `/rosout` - ROS logging

Use `ros2 topic list` to see all currently active topics.

## 📦 Data Collection with ROS2 Bags

### **Record All Sensor Data in One Bag**

```bash
# Create bags directory
mkdir -p ~/helmet_bags
cd ~/helmet_bags

# Record all helmet sensor data for 60 seconds
ros2 bag record -o helmet_complete_$(date +%Y%m%d_%H%M%S) \
  /zed2i/zed_node/rgb/color/rect/image/compressed \
  /zed2i/zed_node/depth/depth_registered/compressed \
  /zed2i/zed_node/imu/data \
  /zed2i/zed_node/odom \
  /zed2i/zed_node/pose \
  /audio \
  /audio/channel0 /audio/channel1 /audio/channel2 /audio/channel3 /audio/channel4 /audio/channel5 \
  /doa /speech_audio /vad \
  /rslidar_points \
  /imu_node/cali \
  /tf /tf_static \
  /diagnostics \
  --max-bag-duration 60
```

### **Quick Recording Commands**

```bash
# Record everything for 30 seconds
ros2 bag record -a -o helmet_all_data --max-bag-duration 30

# Record only vision + transforms (smaller file)
ros2 bag record -o helmet_vision \
  /zed2i/zed_node/rgb/color/rect/image/compressed \
  /zed2i/zed_node/depth/depth_registered/compressed \
  /zed2i/zed_node/odom /zed2i/zed_node/pose \
  /tf /tf_static

# Record audio analysis only
ros2 bag record -o helmet_audio \
  /doa /speech_audio /vad /audio
```

### **Playback Recorded Data**

```bash
# List bag contents
ros2 bag info helmet_complete_YYYYMMDD_HHMMSS

# Play back recorded data
ros2 bag play helmet_complete_YYYYMMDD_HHMMSS

# Play at different speed
ros2 bag play helmet_complete_YYYYMMDD_HHMMSS --rate 0.5  # Half speed
```
