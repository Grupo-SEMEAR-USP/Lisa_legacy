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

    # Clone legged_control
    #git clone -b master git@github.com:lomcin/legged_control.git $WS_SRC_FOLDER/legged_control
    # Clone OCS2
    #git clone https://github.com/leggedrobotics/ocs2.git $WS_SRC_FOLDER/ocs2
    # Clone pinocchio
    #git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git $WS_SRC_FOLDER/pinocchio
    # Clone hpp-fcl
    #git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git $WS_SRC_FOLDER/hpp-fcl
    # Clone ocs2_robotic_assets
    #git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git $WS_SRC_FOLDER/ocs2_robotic_assets
    # Clone pronto
    #git clone https://github.com/ori-drs/pronto.git $WS_SRC_FOLDER/pronto
    # Clone kindr
    #git clone -b master https://github.com/ANYbotics/kindr.git $WS_SRC_FOLDER/kindr
    # Clone kindr ros
    #git clone -b master https://github.com/ANYbotics/kindr_ros.git $WS_SRC_FOLDER/kindr_ros
    # Clone grid map
    #git clone -b master https://github.com/ANYbotics/grid_map.git $WS_SRC_FOLDER/grid_map
    # Clone elevation mapping
    #git clone -b master https://github.com/ANYbotics/elevation_mapping.git $WS_SRC_FOLDER/elevation_mapping
    # Clone realsense gazebo plugin
    #git clone -b melodic-devel https://github.com/pal-robotics/realsense_gazebo_plugin.git $WS_SRC_FOLDER/realsense_gazebo_plugin

    echo "Building $1 docker image..."

    ./docker/build.sh $1

fi
