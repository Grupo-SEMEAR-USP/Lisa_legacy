#!/bin/bash
if [ -z $1 ]; then
    echo "Please, give me a ROS image name."
else
    if [ -z $2 ]; then
        echo "Please, give me a user name. Otherwise, put 'catkin' as username."
    else
        if [[ "$2" == "catkin" ]]; then
            CONTAINER_LABEL="ros_$1"
        else
            CONTAINER_LABEL="ros_$1-$2"
        fi

        docker start $CONTAINER_LABEL

        docker exec -it \
        --user $(id -u):$(id -g) \
        $CONTAINER_LABEL ${@:3}
    fi
fi
