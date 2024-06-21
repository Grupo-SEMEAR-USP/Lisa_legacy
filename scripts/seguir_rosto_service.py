#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
from face_direction_pkg.srv import FaceCenter, FaceCenterRequest

class FaceCenterClient:
    def __init__(self):
        rospy.loginfo("Initializing FaceCenterClient")
        self.bridge = CvBridge()
        rospy.loginfo("CvBridge initialized")
        self.image_sub = rospy.Subscriber("/Imagens", Image, self.image_callback)
        rospy.loginfo("Subscriber initialized")
        self.center_x_pub = rospy.Publisher("/face_center_x", Float32, queue_size=10)
        self.center_y_pub = rospy.Publisher("/face_center_y", Float32, queue_size=10)
        rospy.loginfo("Publishers initialized")
        rospy.wait_for_service('face_center')
        self.face_center_service = rospy.ServiceProxy('face_center', FaceCenter)
        rospy.loginfo("Service proxy initialized")

    def image_callback(self, msg):
        rospy.loginfo("Received image message")
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            rospy.loginfo("Image converted to OpenCV format")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        # Cria a requisição para o serviço
        request = FaceCenterRequest(image=msg)

        try:
            # Chama o serviço
            response = self.face_center_service(request)
            rospy.loginfo("Service call successful")

            # Publica as coordenadas do centro do rosto
            self.center_x_pub.publish(Float32(response.center_x))
            self.center_y_pub.publish(Float32(response.center_y))
            rospy.loginfo(f"Published center_x={response.center_x}, center_y={response.center_y}")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

def main():
    rospy.init_node('face_center_client')
    face_center_client = FaceCenterClient()
    rospy.loginfo("FaceCenterClient node initialized, spinning...")
    rospy.spin()

if __name__ == '__main__':
    main()
