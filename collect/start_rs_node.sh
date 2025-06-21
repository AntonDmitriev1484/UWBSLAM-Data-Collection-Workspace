#!/usr/bin/env bash

cd /opt/ros/humble/share/realsense2_camera/launch/

source /opt/ros/humble/setup.bash

ros2 launch realsense2_camera rs_launch.py \
device_type:=d435 \
enable_sync:=true \
enable_gyro:=true \
gyro_fps:=200 \
enable_accel:=true \
accel_fps:=200 \
unite_imu_method:=2 \
enable_color:=true \
enable_depth:=true \
enable_infra1:=false \
enabel_infra2:=false \
rgb_camera.color_profile:="640,480,30" \
depth_module.depth_profile:="640,480,30" \
#align_depth.enable:=true
  
