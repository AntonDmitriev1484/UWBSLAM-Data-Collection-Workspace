#!/usr/bin/env bash

cd /home/admi3ev/Beluga-Firmware-Mod/ROS/

source ./install/setup.bash

ros2 run beluga beluga \
  --ros-args \
  --param ranges_name:=uwb_ranges3 \
  --param exchange_name:=uwb_exchanges3 \
  --param port:="/dev/ttyACM1" \
  --param config:="./config3.json"
