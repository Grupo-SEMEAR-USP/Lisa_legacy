FROM osrf/ros:noetic-desktop-focal

# Avoiding interactive problems when updating
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Sao_Paulo

RUN apt update && apt upgrade -y

RUN apt install -y ros-noetic-rviz
# RUN apt install -y gazebo11