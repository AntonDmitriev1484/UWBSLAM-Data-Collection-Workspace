#!/bin/bash

# Usage check
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <bag_filename no .bag>"
    exit 1
fi

BAG_FILE="$1"

rosrun kalibr kalibr_calibrate_imu_camera \
    --bag "/data/collect/ros1/${BAG_FILE}.bag" \
    --target ./april_6x6_config.yaml \
	--imu /data/allan_var/out/imu_scaled.yaml \
	--imu-models calibrated \
	--cam "./cam_out/${BAG_FILE}-camchain.yaml" \

