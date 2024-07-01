# Instalação e Configuração do rosserial para ROS Noetic

Este guia fornece instruções passo a passo para instalar e configurar o rosserial no ROS Noetic.

## Passos para Instalação

1. Atualize a lista de pacotes:

   sudo apt-get update

2. Instale os pacotes rosserial e rosserial-arduino:

   sudo apt-get install -y ros-noetic-rosserial-arduino ros-noetic-rosserial

3. Clone o repositório rosserial no seu workspace do Catkin:

   cd ~/catkin_ws/src
   git clone https://github.com/ros-drivers/rosserial.git

4. Compile o workspace do Catkin:

   cd ..
   catkin_make

5. Fonte o setup do Catkin:

   . devel/setup.bash

6. Gere as bibliotecas para Arduino:

   cd ~/catkin_ws/src/rosserial/rosserial_arduino/src/rosserial_arduino
   rosrun rosserial_arduino make_libraries.py .

7. Copie a biblioteca gerada para a pasta de bibliotecas do Arduino:

   cp -r ~/catkin_ws/src/rosserial/rosserial_arduino/src/rosserial_arduino/ros_lib ~/Arduino/libraries

8. Execute o nó serial do rosserial:

   rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
