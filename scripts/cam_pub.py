#!/usr/bin/python3
# Basics ROS program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

def find_camera_index(max_index=10):
    """
    Find the first camera index that has a valid image.
    """
    for index in range(max_index):
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            ret, _ = cap.read()
            cap.release()
            if ret:
                return index
    return None

def publish_message():
    # Find a valid camera index
    camera_index = find_camera_index()
    if camera_index is None:
        rospy.logerr("Não foi encontrada nenhuma câmera disponível.")
        return

    # Node is publishing to the /Imagens topic using 
    # the message type Image
    pub = rospy.Publisher('/Imagens', Image, queue_size=10)
     
    # Tells rospy the name of the node.
    # Node name is set to Receber_Imagem
    rospy.init_node('Receber_Imagem', anonymous=False)
     
    # Go through the loop 10 times per second
    rate = rospy.Rate(10) # 10hz
     
    # Create a VideoCapture object with the found camera index
    cap = cv2.VideoCapture(camera_index)
     
    # Used to convert between ROS and OpenCV images
    br = CvBridge()
    # Desired resolution
    desired_width = 320
    desired_height = 240

    # Buffer settings
    buffer_size = 10  # Publish every 10 frames
    frame_count = 0

    # While ROS is still running.
    while not rospy.is_shutdown(): 
     
        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, frame = cap.read()
         
        if ret == True:
            # Increment the frame count
            frame_count += 1

            # Publish only if the frame count matches the buffer size
            if frame_count >= buffer_size:
                frame_count = 0  # Reset the frame count
                # Print debugging information to the terminal
                #rospy.loginfo('Publicando video frame')
                resized_frame = cv2.resize(frame, (desired_width, desired_height))
                 
                # Publish the image.
                # The 'cv2_to_imgmsg' method converts an OpenCV
                # image to a ROS image message
                # cv2.imshow("Imagem Original", frame)
                # cv2.waitKey(1)
                pub.publish(br.cv2_to_imgmsg(resized_frame, "bgr8"))
             
        # Sleep just enough to maintain the desired rate
        rate.sleep()
         
if __name__ == '__main__':
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass
