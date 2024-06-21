#!/usr/bin/python3
# Description:
# - Subscribes to real-time streaming video from your built-in webcam.
# - Sends commands to a serial device based on the received video stream data.
#
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy
from sensor_msgs.msg import Image
import serial
import time
# If you need to convert ROS Image messages to OpenCV format uncomment the next line
# from cv_bridge import CvBridge
 
def callback(data):
  rospy.loginfo("Receiving finger count")
  contador = int(data.data)  # Assuming data contains the finger count as integer

  # Here, add your conditions based on the value of 'contador'
  if contador == 1:
      command = 'PARAR'
  elif contador == 2:
      command = 'HORARIO'
  elif contador == 3:
      command = 'ANTIHORARIO'
  else:
      command = 'NOP'  # No operation

  # Send command to the serial port
  ser.write(command.encode('utf-8') + b'\n')
  rospy.loginfo(f"Sent command: {command}")
   
def receive_message():
  rospy.init_node('Node_Serial', anonymous=False)
  rospy.Subscriber('finger_count', Image, callback)
  rospy.spin()
  # cv2.destroyAllWindows()  # Uncomment if using OpenCV to handle video windows
  
if __name__ == '__main__':
  ser = serial.Serial('/dev/ttyUSB0', 115200)  # Open the serial port at 115200 baud
  try:
      receive_message()
  except rospy.ROSInterruptException:
      pass  # Handle ROS interrupt exception gracefully
  except KeyboardInterrupt:
      ser.close()  # Close the serial port when exiting the program
