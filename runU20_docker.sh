#!/usr/bin/env bash

FOLDER=./
xhost +local:root

# Remove existing container
docker rm -f U20 2>/dev/null

# Start container in detached mode (-d)
docker run -dit --name U20 -e "DISPLAY=$DISPLAY" -e "QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "$FOLDER:/data" u20_tools:latest

# Wait a bit to ensure container is fully started
sleep 1

# Launch two terminal windows and run docker exec
gnome-terminal -- bash -c "docker exec -it U20 bash"
gnome-terminal -- bash -c "docker exec -it U20 bash"

