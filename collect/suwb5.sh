#!/usr/bin/env bash

cd /home/admi3ev/Beluga-Firmware-Mod/ROS/

source ./install/setup.bash

ros2 run beluga beluga \
  --ros-args \
  --param ranges_name:=uwb_ranges \
  --param exchange_name:=uwb_exchanges \
  --param port:="/dev/ttyACM0" \
  --param config:="./config5.json"
