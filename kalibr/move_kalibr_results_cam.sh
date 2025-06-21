#!/bin/bash

# Source and destination directories
SRC_DIR=~/ws/collect/ros1
DST_DIR=~/ws/kalibr/cam_out

# Ensure destination directory exists
mkdir -p "$DST_DIR"

# Move all files that do not contain '.bag' in their filename
find "$SRC_DIR" -maxdepth 1 -type f ! -name "*.bag*" -exec mv {} "$DST_DIR" \;
