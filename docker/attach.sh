#!/bin/bash

if [ -z $1 ]; then
    echo "Please, give me a ROS image name."
else
    if [ -z $2 ]; then
        CONTAINER_LABEL=ros_$1
    else
        CONTAINER_LABEL="ros_$1-$2"
    fi

    CONTAINER_EXIST=0

    if [ -z $(docker container ls -a --format="{{.Names}}" --filter name=^$CONTAINER_LABEL$) ]; then
        ./docker/run_detached.sh $1 $2
    fi

    docker start $CONTAINER_LABEL
    docker exec -it $CONTAINER_LABEL "/bin/bash"
fi