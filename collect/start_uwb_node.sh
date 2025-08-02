#!/usr/bin/env bash


# Usage info
if [ -z "$1" ]; then
  echo "Usage: $0 <ACM_port_number>"
  echo "Example: $0 2  # for /dev/ttyACM2"
  exit 1
fi

ACM_PORT="/dev/ttyACM$1"

cd /home/admi3ev/Beluga-Firmware-Mod/ROS/

source ./install/setup.bash

ros2 run beluga beluga \
  --ros-args \
  --param ranges_name:=uwb_ranges \
  --param exchange_name:=uwb_exchanges \
  --param port:=$ACM_PORT \
  --param config:=./config.json
