#!/bin/bash
# Helmet Data Recording Script
# Usage: ./record_helmet_data.sh [duration_in_seconds]

DURATION=${1:-30}  # Default 30 seconds
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT_DIR=~/helmet_bags
FILENAME="helmet_complete_${TIMESTAMP}"

echo "🎥 Recording helmet sensor data for ${DURATION} seconds..."
echo "📁 Output: ${OUTPUT_DIR}/${FILENAME}"

mkdir -p ${OUTPUT_DIR}

ros2 bag record -o ${OUTPUT_DIR}/${FILENAME} \
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
  /rosout \
  --max-bag-duration ${DURATION}

echo "✅ Recording complete! Bag saved to: ${OUTPUT_DIR}/${FILENAME}"
echo "🔍 To inspect: ros2 bag info ${OUTPUT_DIR}/${FILENAME}"
echo "▶️  To replay: ros2 bag play ${OUTPUT_DIR}/${FILENAME}"