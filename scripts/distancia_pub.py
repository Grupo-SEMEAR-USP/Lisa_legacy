#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32
import mediapipe as mp

def calcular_distancia(face_width, focal_length, pixel_width):
    return (face_width * focal_length) / pixel_width

# Matriz de calibração da minha câmera
camera_matrix = np.array([[6.73013110e+02, 0.00000000e+00, 1.74499999e+02],
                          [0.00000000e+00, 6.72565563e+02, 1.49500000e+02],
                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

# Coeficientes de distorção da minha câmera
dist_coeffs = np.array([[-1.20362885e-01, 4.78388299e-02, -2.92259372e-03, -6.27378830e-03, 1.16195179e-02]])

average_face_width = 14.0 

class DistancePublisher:
    def __init__(self):
        rospy.init_node('distance_publisher_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/Imagens', Image, self.image_callback)
        self.distance_pub = rospy.Publisher('/face_distance', Float32, queue_size=10)
        self.face_detection = mp.solutions.face_detection.FaceDetection(min_detection_confidence=0.5)

    def image_callback(self, data):
        try:
            if data.encoding == "8UC3":
                rospy.logwarn(f"Unexpected image encoding: {data.encoding}. Converting to 'bgr8'.")
                # Converte a mensagem ROS para uma imagem OpenCV com tipo '8UC3'
                cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            else:
                # Converte a mensagem ROS para uma imagem OpenCV
                cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")

            # Detecta rostos na imagem
            faces = self.detect_faces(cv_image)

            if faces is not None and len(faces) > 0:
                # Calcular a distância para cada rosto e encontrar o mais próximo dentro do limite
                min_distance = float('inf')
                for (x, y, w, h) in faces:
                    focal_length = camera_matrix[0, 0]  # Distância focal (fx)
                    face_width_in_pixels = w  # Largura do rosto em pixels
                    distance = calcular_distancia(average_face_width, focal_length, face_width_in_pixels)

                    # Nessa função eu coloquei para ele publicar apenas a menor distância, testei e deu certo
                    if distance < min_distance and distance <= 100:  # Considerar apenas distâncias até 1 metro
                        min_distance = distance

                # Defini a distância máxima para ele publicar como sendo 100cm
                if min_distance <= 100:
                    self.distance_pub.publish(min_distance)
        except Exception as e:
            rospy.logerr(f"Erro ao processar a imagem: {e}")

    def detect_faces(self, image):
        # Converte a imagem para RGB pra usar o mediapipe
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        results = self.face_detection.process(rgb_image)

        faces = []
        if results.detections:
            for detection in results.detections:
                bboxC = detection.location_data.relative_bounding_box
                ih, iw, _ = image.shape
                bbox = (int(bboxC.xmin * iw), int(bboxC.ymin * ih),
                        int(bboxC.width * iw), int(bboxC.height * ih))
                faces.append(bbox)

        return faces

if __name__ == '__main__':
    try:
        distance_publisher = DistancePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
