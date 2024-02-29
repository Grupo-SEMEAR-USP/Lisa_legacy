#!/bin/bash

if [ -z $1 ]; then
    echo "Please, give me a ROS name. Check dockerfiles in docker folder."

else
    if [ -z $2 ]; then
        USER_NAME=catkin
        CONTAINER_ALIAS=$1
    else
        USER_NAME=$2
        CONTAINER_ALIAS=$1-$2
    fi

    CONTAINER_HOME=docker/container/$CONTAINER_ALIAS/home/$USER
    WS_SRC_FOLDER=$CONTAINER_HOME/catkin_ws/src
    SPLITED_ONE=($(echo $1 | tr "-" "\n"))

    echo "source /opt/ros/${SPLITED_ONE[0]}/setup.bash" >> $CONTAINER_HOME/.bashrc

    mkdir -p $WS_SRC_FOLDER

    echo "This may take a while... Downloading needed packages' repositories..."

    #git clone -b master https://github.com/catkin/catkin_simple.git $WS_SRC_FOLDER/catkin_simple
    #git clone -b master https://github.com/ethz-asl/eigen_checks.git $WS_SRC_FOLDER/eigen_checks
    #git clone -b main git@github.com:viviansuzano/go1.git $WS_SRC_FOLDER/go1
    #git clone -b master https://github.com/robotology/osqp-eigen.git $WS_SRC_FOLDER/osqp-eigen
    #git clone -b master https://github.com/viviansuzano/pronto.git $WS_SRC_FOLDER/pronto
    #git clone -b melodic-devel https://github.com/viviansuzano/rbdl.git $WS_SRC_FOLDER/rbdl
    #git clone -b master https://github.com/viviansuzano/xpp.git $WS_SRC_FOLDER/xpp
    #git clone -b main git@github.com:viviansuzano/xpp_go1.git $WS_SRC_FOLDER/xpp_go1

    echo "Building $1 docker image..."

    ./docker/build.sh $1

fi