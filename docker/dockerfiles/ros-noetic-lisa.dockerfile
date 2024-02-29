FROM osrf/ros:noetic-desktop-full-focal

# Avoiding interactive problems when updating
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Sao_Paulo

RUN apt update && apt upgrade -y

# Install dependencies
RUN apt install -y ros-noetic-rviz
RUN apt install wget git build-essential -y
RUN apt-get install python3-catkin-tools -y
RUN apt install liburdfdom-dev liboctomap-dev libassimp-dev
RUN apt install ros-noetic-tf2-tools
