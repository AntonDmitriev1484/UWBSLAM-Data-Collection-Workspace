#!/bin/bash

# Check if an argument was provided
if [ $# -lt 1 ]; then
  echo "Usage: $0 <bagname>"
  exit 1
fi

BAGNAME=$1

rosbags-convert \
  --src "ros2/${BAGNAME}/" \
  --dst "ros1/${BAGNAME}.bag" \
  --include-topic /camera/camera/imu \
  --include-topic /camera/camera/infra2/image_rect_raw \
  --include-topic /camera/camera/infra1/image_rect_raw
