#!/bin/bash

# Allow local Docker GUI access
xhost +local:root

# Get the correct username whether or not running with sudo
if [ -z "$SUDO_USER" ]
then
    user=$USER
else
    user=$SUDO_USER
fi

# Run the container
docker run -it \
    --name=ur5_planners_ros1 \
    --shm-size=1g \
    --ulimit memlock=-1 \
    --device=/dev/bus/usb \
    --group-add plugdev \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix" \
    --volume="/home/$user/miapr/Projekt/MIAPR_DOCKER/ros1:/root/Shared/trajopt_ws:rw" \
    --env="DISPLAY=$DISPLAY" \
    --network=host \
    --privileged \
    ur5_planners_dev \
    bash
