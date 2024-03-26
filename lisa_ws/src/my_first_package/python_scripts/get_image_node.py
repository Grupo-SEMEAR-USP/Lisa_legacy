#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Definindo variáveis e objetos
nodeName, topicName = 'get_image_node', '/cam_image'
bridge = CvBridge()

# Iniciando nó para pegar imagem
rospy.init_node(nodeName, anonymous = True)
pub = rospy.Publisher(topicName, Image, queue_size = 1)
rospy.loginfo("get_image_node started")

# Capturando imagem da camera
cap = cv2.VideoCapture(0)
if cap.isOpened():
    rospy.loginfo("Camera accessed successfully")
else:
    rospy.logerr("Camera cannot be accessed")


def get_image():
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break

        msg = bridge.cv2_to_imgmsg(frame,'bgr8')   
        pub.publish(msg) 
        cv2.imshow("Camera", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if rospy.is_shutdown():
            cap.release()         

if __name__ == '__main__':
    try:
        get_image()
    except rospy.ROSInterruptException:
        pass

