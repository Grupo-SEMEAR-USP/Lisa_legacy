#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

nodeName, topicName = 'show_image_node', '/face_image'

bridge = CvBridge()

def callbackfunc(message):
    frame = bridge.imgmsg_to_cv2(message)
    cv2.imshow("Rosto", frame)
    cv2.waitKey(1)

rospy.init_node(nodeName, anonymous=True)
rospy.Subscriber(topicName, Image, callbackfunc)
rospy.spin()

cv2.destroyAllWindows()