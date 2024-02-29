FROM nvidia/cudagl:11.4.2-runtime-ubuntu18.04

# Avoiding interactive problems when updating
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Recife

RUN apt update && apt upgrade -y

RUN apt install -y wget git build-essential lsb-release curl

RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update
RUN apt-get install -y python3-catkin-tools

RUN apt-get install -y ros-melodic-desktop-full

# INSTALLING EIGEN 3 FROM SOURCE
WORKDIR /opt
RUN git clone -b 3.4 https://gitlab.com/libeigen/eigen.git
RUN cd eigen && mkdir build && cd build && cmake .. 
WORKDIR /opt/eigen/build
RUN make install

# INSTALLING OSQP
WORKDIR /opt
RUN git clone --recursive https://github.com/osqp/osqp.git

RUN cd osqp && mkdir build && cd build && cmake -G "Unix Makefiles" ..

WORKDIR /opt/osqp/build
RUN cmake --build . --target install

RUN apt-get install -y ros-melodic-joint-state-publisher-gui
RUN apt-get install -y ros-melodic-rqt-multiplot

RUN apt-get install -y mesa-utils libgl1-mesa-glx

# IF YOU NEED TO DEBUG CODE
# RUN apt install gdb -y