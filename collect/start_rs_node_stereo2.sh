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
enable_infra1:=true \
enable_infra2:=true \
rgb_camera.color_profile:="640,480,30" \
depth_module.depth_profile:="640,480,30" \
align_depth.enable:=true \
depth_module.infra_profile:="640,480,30" \
depth_module.emitter_enabled:=0


