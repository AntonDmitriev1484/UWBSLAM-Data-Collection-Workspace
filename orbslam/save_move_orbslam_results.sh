#!/bin/bash

# Check for input argument
if [ -z "$1" ]; then
  echo "Usage: $0 <filename_prefix>"
  exit 1
fi

A="$1"

# Call the ROS service
echo "Calling ROS service..."
output=$(rosservice call /orb_slam3/save_traj "$A")

# Wait until we get success: True
if echo "$output" | grep -q "success: True"; then
  echo "ROS service succeeded."

  # Define file names
  cam_file="/root/.ros/${A}_cam_traj.txt"
  kf_file="/root/.ros/${A}_kf_traj.txt"

  # Move the files
  dest_dir="/data/orbslam/out"
  mkdir -p "$dest_dir"

  mv "$cam_file" "$dest_dir/" && echo "Moved $cam_file to $dest_dir"
  mv "$kf_file" "$dest_dir/" && echo "Moved $kf_file to $dest_dir"
else
  echo "ROS service failed or did not return success: True"
  exit 2
fi
