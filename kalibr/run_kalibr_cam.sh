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
    --models pinhole-radtan pinhole-radtan \
    --topics /camera/camera/infra1/image_rect_raw /camera/camera/infra2/image_rect_raw

