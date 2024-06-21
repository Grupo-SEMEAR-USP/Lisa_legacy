#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()
timeout_duration = rospy.Duration(5)  # Duração do timeout em segundos
last_image_time = None

def face_callback(msg):
    global last_image_time
    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Rosto Detectado", frame)
        cv2.waitKey(1)
        last_image_time = rospy.Time.now()
    except Exception as e:
        rospy.logerr(f"Erro ao exibir a imagem: {e}")

def check_timeout(event):
    global last_image_time
    if last_image_time and (rospy.Time.now() - last_image_time > timeout_duration):
        cv2.destroyAllWindows()
        rospy.signal_shutdown("Tópico 'rostos' não atualizado por muito tempo.")

def main():
    global last_image_time
    rospy.init_node('face_viewer')
    last_image_time = rospy.Time.now()
    rospy.Subscriber('rostos', Image, face_callback)
    rospy.Timer(rospy.Duration(1), check_timeout)  # Verifica o timeout a cada segundo
    rospy.spin()

if __name__ == "__main__":
    main()
