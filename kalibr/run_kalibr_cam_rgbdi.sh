#!/bin/bash

# Usage check
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <bag_filename no .bag>"
    exit 1
fi

BAG_FILE="$1"

rosrun kalibr kalibr_calibrate_cameras \
    --bag "/data/collect/ros1/${BAG_FILE}.bag" \
    --target /data/kalibr/april_6x6_config.yaml \
    --models pinhole-radtan \
    --topics /camera/camera/color/image_raw

