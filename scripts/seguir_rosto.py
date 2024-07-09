#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
import mediapipe as mp
from cv_bridge import CvBridge, CvBridgeError

class FaceDirectionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.face_detection = mp.solutions.face_detection.FaceDetection(min_detection_confidence=0.2)
        self.image_sub = rospy.Subscriber("/Imagens", Image, self.image_callback)
        self.center_x_pub = rospy.Publisher("/face_center_x", Float32, queue_size=10)
        self.center_y_pub = rospy.Publisher("/face_center_y", Float32, queue_size=10)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.face_detection.process(image_rgb)

        if results.detections:
            num_faces = len(results.detections)
            rospy.loginfo(f"Detected {num_faces} faces")

            # Encontra a detecção com a maior área de bounding box
            largest_detection = max(results.detections, key=lambda det: det.location_data.relative_bounding_box.width * det.location_data.relative_bounding_box.height)

            bbox = largest_detection.location_data.relative_bounding_box
            height, width, _ = frame.shape
            x_center = int(bbox.xmin * width + (bbox.width * width) / 2)
            y_center = int(bbox.ymin * height + (bbox.height * height) / 2)

            rect_center_x = width // 2
            rect_center_y = height // 2
            distance_x = x_center - rect_center_x
            distance_y = y_center - rect_center_y

            rospy.loginfo(f"Distance to closest face: center_x={distance_x}, center_y={distance_y}")

            self.center_x_pub.publish(Float32(distance_x))
            self.center_y_pub.publish(Float32(distance_y))
        else:
            rospy.loginfo("No faces detected")

def main():
    rospy.init_node('face_direction_node')
    face_direction_node = FaceDirectionNode()
    rospy.spin()

if __name__ == '__main__':
    main()
