#!/bin/bash
if [ -z $1 ]; then
    echo "Please, give me a ROS image name."
else
    if [ -z $2 ]; then
        USER_NAME=catkin
        CONTAINER_LABEL=ros_$1
        CONTAINER_ALIAS=$1
    else
        USER_NAME=$2
        CONTAINER_LABEL="ros_$1-$2"
        CONTAINER_ALIAS="$1-$2"
    fi

    # Container paths
    CONTAINER_USER_HOME=/home/$USER
    CONTAINER_WORKDIR=$CONTAINER_USER_HOME/catkin_ws
    CONTAINER_SCRIPTS=$CONTAINER_WORKDIR/scripts

    # Host paths
    HOST_USER_HOME=$(pwd)/docker/container/$CONTAINER_ALIAS/home/$USER
    HOST_WORKDIR=$HOST_USER_HOME/catkin_ws
    HOST_SCRIPTS=$(pwd)/scripts

    # Creating useful folders in host
    mkdir -p $HOST_USER_HOME
    mkdir -p $HOST_WORKDIR

    # Automatically detect NVIDIA's GPU and try to use it
    if [ -z "$(lspci | grep NVIDIA)" ]; then
        USE_GPUS=""
        echo "NVIDIA's GPU WAS NOT detected."
    else
        USE_GPUS="--gpus all"
        echo "NVIDIA's GPU WAS detected. Activating '--gpus all' flag."
    fi

    docker run -di \
        $USE_GPUS \
        --user $(id -u):$(id -g) \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --network="host" \
        --ipc="host" \
        --privileged \
        --oom-kill-disable \
        --volume="/etc/group:/etc/group:ro" \
        --volume="$HOST_USER_HOME:$CONTAINER_USER_HOME:rw" \
        --volume="/etc/passwd:/etc/passwd:ro" \
        --volume="/etc/shadow:/etc/shadow:ro" \
        --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="$HOST_WORKDIR:$CONTAINER_WORKDIR:rw" \
        --volume="$HOST_SCRIPTS:$CONTAINER_SCRIPTS:rw" \
        --name="$CONTAINER_LABEL" \
        --volume="/dev:/dev:rw" \
        --workdir="$CONTAINER_WORKDIR" \
        semear/lisa-ws:ros-$1
        # --volume="/dev/shm:/dev/shm:rw" \
fi