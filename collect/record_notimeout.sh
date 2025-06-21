#!/usr/bin/env bash

if [ $# -lt 2 ]; then
  echo "Usage: $0 <delay_in_seconds> <duration_in_seconds> <bag_name>"
  exit 1
fi

cd ros2/

DELAY=$1
BAG_NAME=$2

for ((i=DELAY; i>0; i--)); do
  echo "Starting in $i..."
  sleep 1
done

echo "Recording all topics to bag named '${BAG_NAME}' for ${DURATION} seconds."
exec  ros2 bag record --all -o "${BAG_NAME}"
