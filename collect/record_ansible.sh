#!/usr/bin/env bash

if [ $# -lt 1 ]; then
  echo "Usage: $0 <bag_name>"
  exit 1
fi

source /opt/ros/humble/setup.bash

cd ros2/

BAG_NAME=$1

echo "Recording all topics to bag named '${BAG_NAME}' for ${DURATION} seconds."
ros2 bag record --all -o "${BAG_NAME}"

# Mark that the script is running
touch ./runningtest.txt
