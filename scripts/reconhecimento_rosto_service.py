#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger, TriggerResponse
import cv2
import mediapipe as mp

bridge = CvBridge()
mp_face_detection = mp.solutions.face_detection
face_detection = mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.5)

class FaceRecognizer:
    def __init__(self):
        self.image = None
        self.latest_face = None
        self.face_pub = rospy.Publisher('rostos', Image, queue_size=10)
        rospy.Subscriber('/Imagens', Image, self.image_callback)
        self.service = rospy.Service('recognize_face', Trigger, self.handle_recognize_face)
        rospy.loginfo("Serviço de reconhecimento de rosto iniciado.")

    def image_callback(self, msg):
        self.image = msg

    def handle_recognize_face(self, req):
        self.process_image()
        response = TriggerResponse()
        response.success = self.latest_face is not None
        response.message = "Rosto detectado" if self.latest_face is not None else "Nenhum rosto detectado"
        return response

    def process_image(self):
        if self.image is not None:
            try:
                frame = bridge.imgmsg_to_cv2(self.image, desired_encoding='passthrough')
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = face_detection.process(rgb_frame)

                if results.detections:
                    max_area = 0
                    closest_face = None
                    for detection in results.detections:
                        bboxC = detection.location_data.relative_bounding_box
                        ih, iw, _ = frame.shape
                        bbox = (int(bboxC.xmin * iw), int(bboxC.ymin * ih),
                                int(bboxC.width * iw), int(bboxC.height * ih))
                        area = bbox[2] * bbox[3]
                        if area > max_area:
                            max_area = area
                            closest_face = frame[bbox[1]:bbox[1] + bbox[3], bbox[0]:bbox[0] + bbox[2]]
                    self.latest_face = closest_face
                    if self.latest_face is not None:
                        self.face_pub.publish(bridge.cv2_to_imgmsg(self.latest_face, "bgr8"))
                    rospy.loginfo("Rosto detectado")
                else:
                    self.latest_face = None
            except Exception as e:
                rospy.logerr(f"Erro ao processar imagem: {e}")

if __name__ == "__main__":
    rospy.init_node('recognize_face_server')
    FaceRecognizer()
    rospy.spin()
