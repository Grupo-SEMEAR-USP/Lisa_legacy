FROM osrf/ros:melodic-desktop-full-bionic

# Avoiding interactive problems when updating
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Recife

RUN apt update && apt upgrade -y

RUN apt install wget git build-essential -y

RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

RUN apt-get update
RUN apt-get install python3-catkin-tools -y

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

RUN apt-get install ssh

RUN ssh-keygen -t rsa

# IF YOU NEED TO DEBUG CODE
# RUN apt install gdb -y