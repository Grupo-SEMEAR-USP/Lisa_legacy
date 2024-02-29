#!/bin/bash

if [ -z $1 ]; then
    echo "Please, give me a ROS image name."
else
    DOCKER_FOLDER=docker/dockerfiles

    if [ -e $DOCKER_FOLDER/ros-$1.dockerfile ]; then
        docker build -f $DOCKER_FOLDER/ros-$1.dockerfile -t semear/lisa-ws:ros-$1 .
    else
        echo "Please, give me a VALID ROS image name."
        echo "The available images are: "
        ls $DOCKER_FOLDER -w 1 | sed 's/\(^ros\)\-\([a-z A-Z 0-9\-]*\)\.\([a-z]*\)/\ \ \2/'
    fi
fi