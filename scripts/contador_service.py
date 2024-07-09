#!/usr/bin/env python3
import os
os.environ['GLOG_minloglevel'] = '2'  # Suprimir mensagens de log de inicialização do MediaPipe

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger, TriggerResponse
import cv2
import mediapipe as mp

bridge = CvBridge()
mp_hands = mp.solutions.hands

class FingerCounter:
    def __init__(self, interval=0.7):
        self.image = None
        self.processing = False
        self.latest_finger_count = 0
        self.finger_count_streak = {}
        self.hands = mp_hands.Hands(static_image_mode=True, model_complexity=0, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        rospy.Subscriber('/Imagens', Image, self.image_callback)
        self.timer = rospy.Timer(rospy.Duration(interval), self.process_image)
        self.service = rospy.Service('/get_finger_count', Trigger, self.handle_get_finger_count)

    def image_callback(self, msg):
        self.image = msg

    def process_image(self, event):
        if rospy.get_param('/stop_counting', False):  # Verifica se a contagem de dedos deve estar desativada
            return
        if self.image is not None and not self.processing:
            self.processing = True
            try:
                frame = bridge.imgmsg_to_cv2(self.image, desired_encoding='passthrough')
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = self.hands.process(frame_rgb)
                total_finger_count = 0

                if results.multi_hand_landmarks:
                    for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                        hand_landmarks_list = [[lm.x, lm.y] for lm in hand_landmarks.landmark]
                        label = handedness.classification[0].label

                        # Outros dedos
                        if hand_landmarks_list[8][1] < hand_landmarks_list[6][1] and hand_landmarks_list[8][0] > hand_landmarks_list[5][0]:  # Indicador (novo critério)
                            total_finger_count += 1
                        if hand_landmarks_list[12][1] < hand_landmarks_list[10][1]:  # Médio
                            total_finger_count += 1
                        if hand_landmarks_list[16][1] < hand_landmarks_list[14][1]:  # Anelar
                            total_finger_count += 1
                        if hand_landmarks_list[20][1] < hand_landmarks_list[18][1]:  # Mínimo
                            total_finger_count += 1

                        # Verificar se apenas o dedo médio está levantado
                        if total_finger_count == 1 and hand_landmarks_list[12][1] < hand_landmarks_list[10][1]:
                            # Verifica se o dedo médio está levantado em relação aos outros dedos
                            if (hand_landmarks_list[12][1] < hand_landmarks_list[8][1] and
                                hand_landmarks_list[12][1] < hand_landmarks_list[16][1] and
                                hand_landmarks_list[12][1] < hand_landmarks_list[20][1]):
                                rospy.loginfo("Dedo do meio levantado")
                                total_finger_count = 11

                        # Polegar (sem necessidade de verificar outros dedos levantados)
                        if total_finger_count != 11:  # Só verifica o polegar se não for o caso do dedo médio levantado
                            if label == 'Left':
                                if hand_landmarks_list[4][0] > hand_landmarks_list[1][0] and hand_landmarks_list[4][1] < hand_landmarks_list[3][1] and total_finger_count>=4:  # Thumb
                                    total_finger_count += 1
                            else:  # Right hand
                                if hand_landmarks_list[4][0] < hand_landmarks_list[3][0] and hand_landmarks_list[4][1] < hand_landmarks_list[2][1] and total_finger_count>=4:  # Thumb
                                    total_finger_count += 1

                self.update_finger_count_streak(total_finger_count)
                        
                rospy.loginfo(f"Contador de dedos: {total_finger_count}")
            except Exception as e:
                rospy.logerr(f"Erro ao processar imagem: {e}")
            finally:
                self.processing = False

    def update_finger_count_streak(self, finger_count):
        if finger_count not in self.finger_count_streak:
            self.finger_count_streak[finger_count] = 0
        self.finger_count_streak[finger_count] += 1

        # Resetar outras contagens
        for count in list(self.finger_count_streak):
            if count != finger_count:
                self.finger_count_streak[count] = 0

        if self.finger_count_streak[finger_count] >= 3:
            self.latest_finger_count = finger_count
        else:
            self.latest_finger_count = 0

    def handle_get_finger_count(self, req):
        if self.latest_finger_count in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]:
            return TriggerResponse(success=True, message=str(self.latest_finger_count))
        else:
            return TriggerResponse(success=False, message=str(self.latest_finger_count))

if __name__ == "__main__":
    rospy.init_node('Contador_node')
    FingerCounter(interval=0.7)  # Ajuste o intervalo conforme necessário
    rospy.spin()
