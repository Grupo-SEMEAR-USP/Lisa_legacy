FROM osrf/ros:melodic-desktop-bionic

# Avoiding interactive problems when updating
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Recife

RUN apt update && apt upgrade -y

RUN apt install -y ros-melodic-rviz