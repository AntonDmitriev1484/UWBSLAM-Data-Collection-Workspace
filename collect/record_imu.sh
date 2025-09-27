#!/usr/bin/env bash

if [ $# -lt 3 ]; then
  echo "Usage: $0 <delay_in_seconds> <duration_in_seconds> <bag_name>"
  exit 1
fi

source /opt/ros/humble/setup.bash

cd ros2/

DELAY=$1
DURATION=$2
BAG_NAME=$3

for ((i=DELAY; i>0; i--)); do
  echo "Starting in $i..."
  sleep 1
done

echo "Recording all topics to bag named '${BAG_NAME}' for ${DURATION} seconds."
exec timeout "${DURATION}s" ros2 bag record --all -o "${BAG_NAME}"
