#!/usr/bin/env bash

# Usage info
if [ -z "$1" ] || [ -z "$2" ]; then
  echo "Usage: $0 <ACM_port_number> <nodeid>"
  echo "Example: $0 2 01  # for /dev/ttyACM2 with config01.json"
  exit 1
fi

ACM_PORT="/dev/ttyACM$1"
NODE_ID="$2"
CONFIG_FILE="./config${NODE_ID}.json"

cd /home/admi3ev/Beluga-Firmware-Mod/ROS/ || exit 1

source ./install/setup.bash

echo $CONFIG_FILE
echo $ACM_PORT

ros2 run beluga beluga \
  --ros-args \
  --param ranges_name:=uwb_ranges \
  --param exchange_name:=uwb_exchanges \
  --param port:=$ACM_PORT \
  --param config:=$CONFIG_FILE
