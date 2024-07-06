#!/usr/bin/env python3
import os
os.environ['GLOG_minloglevel'] = '2'  # Suprimir mensagens de log de inicialização do MediaPipe

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32
import cv2
import numpy as np
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
        self.face_image_pub = rospy.Publisher('/best_face_image', Image, queue_size=10)
        # self.hands_pub = rospy.Publisher('/hands', Image, queue_size=10)
        self.face_detection = mp.solutions.face_detection.FaceDetection(min_detection_confidence=0.5)
        # self.hands_detection = mp.solutions.hands.Hands(static_image_mode=True, max_num_hands=4, min_detection_confidence=0.5, min_tracking_confidence=0.5)

    def image_callback(self, data):
        try:
            # Converte a mensagem ROS para uma imagem OpenCV com tipo 'passthrough'
            cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
            cv_image = cv_image.copy()  # Copia a imagem para torná-la gravável

            faces = self.detect_faces(cv_image)
            # hands = self.detect_hands(cv_image)

            if faces:
                max_area = 0
                best_face = None
                for (x, y, w, h) in faces:
                    if w > 0 and h > 0:  # Verifica se largura e altura são maiores que zero
                        area = w * h
                        if area > max_area:
                            max_area = area
                            best_face = (x, y, w, h)

                if best_face:
                    x, y, w, h = best_face
                    focal_length = camera_matrix[0, 0]  # Distância focal (fx)
                    face_width_in_pixels = w  # Largura do rosto em pixels
                    distance = calcular_distancia(average_face_width, focal_length, face_width_in_pixels)
                    
                    # Publicar apenas se a distância for até 100 cm
                    if distance <= 100:
                        self.distance_pub.publish(distance)

                        # Extrair e publicar a imagem do melhor rosto
                        best_face_image = cv_image[y:y+h, x:x+w]
                        best_face_msg = self.bridge.cv2_to_imgmsg(best_face_image, encoding="passthrough")
                        self.face_image_pub.publish(best_face_msg)

                        #  Registrar a área do rosto
                        # face_area = w * h
                        # rospy.loginfo(f"Área do rosto: {face_area}")

                        # Processar e registrar as áreas das mãos
                        # if hands:
                        #     hands = sorted(hands, key=lambda x: x[2]*x[3], reverse=True)[:2]
                        #     hand_areas = []
                        #     for hand in hands:
                        #         x, y, w, h = hand
                        #         if w > 0 and h > 0:  # Verifica se largura e altura são maiores que zero
                        #             hand_area = w * h
                        #             intersecao = calcular_area_intersecao(best_face, hand)
                        #             hand_area -= intersecao  # Subtrai a área de interseção
                        #             hand_areas.append(hand_area)
                            
                        #     if len(hand_areas) >= 1:
                        #         rospy.loginfo(f"Área da mão 1: {hand_areas[0]}")
                        #     if len(hand_areas) == 2:
                        #         rospy.loginfo(f"Área da mão 2: {hand_areas[1]}")

        except Exception as e:
            rospy.logerr(f"Erro ao processar a imagem: {e}")

    def detect_faces(self, image):
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

    # def detect_hands(self, image):
    #     rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    #     results = self.hands_detection.process(rgb_image)

    #     hands = []
    #     if results.multi_hand_landmarks:
    #         for hand_landmarks in results.multi_hand_landmarks:
    #             x_min, y_min = 1, 1
    #             x_max, y_max = 0, 0
    #             for lm in hand_landmarks.landmark:
    #                 x_min = min(x_min, lm.x)
    #                 y_min = min(y_min, lm.y)
    #                 x_max = max(x_max, lm.x)
    #                 y_max = max(y_max, lm.y)

    #             ih, iw, _ = image.shape
    #             x_min = int(x_min * iw)
    #             y_min = int(y_min * ih)
    #             x_max = int(x_max * iw)
    #             y_max = int(y_max * ih)

    #             hands.append((x_min, y_min, x_max - x_min, y_max - y_min))

    #     return hands

if __name__ == '__main__':
    try:
        distance_publisher = DistancePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
