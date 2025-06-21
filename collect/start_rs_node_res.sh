#!/usr/bin/env bash

cd /opt/ros/humble/share/realsense2_camera/launch/

source /opt/ros/humble/setup.bash

ros2 launch realsense2_camera rs_launch.py \
enable_sync:=true \
enable_gyro:=true \
gyro_fps:=200 \
enable_accel:=true \
accel_fps:=200 \
unite_imu_method:=2 \
enable_color:=true \
enable_depth:=true \
rgb_camera.color_profile:="848,480,30" \
depth_module.depth_profile:="848,480,30"
  
