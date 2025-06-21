#!/usr/bin/env bash

FOLDER=./
xhost +local:root
docker run -it -e "DISPLAY=$DISPLAY" -e "QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "$FOLDER:/data" u20_tools:latest
