#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from std_srvs.srv import Trigger, TriggerResponse
import os
import rospkg
# Obter o caminho do pacote usando rospkg
rospack = rospkg.RosPack()
package_path = rospack.get_path('estrutura')

# Construir o caminho relativo para o arquivo do modelo
model_path = os.path.join(package_path, 'Models', 'gesture_recognizer.task')

base_options = python.BaseOptions(model_asset_path=model_path)
options = vision.GestureRecognizerOptions(base_options=base_options)
recognizer = vision.GestureRecognizer.create_from_options(options)
bridge = CvBridge()
gesture_mapping = {
    'Thumb_Up': 1,
    'Thumb_Down': 2,
    'Open_Palm': 3,
    'Closed_Fist': 4,
    'Victory': 5,
    'Pointing_Up': 6,
}

gesture_counts = {}

def detect_hand_gesture(frame):
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
    recognition_result = recognizer.recognize(mp_image)
    gesture = None

    if recognition_result.gestures:
        top_gesture = recognition_result.gestures[0][0]
        gesture = top_gesture.category_name
        rospy.loginfo("Gesto detectado: {}".format(gesture))
    else:
        rospy.loginfo("Nenhum gesto detectado.")

    return gesture

def image_callback(msg):
    global current_image
    current_image = msg

def handle_gesture_recognition(req):
    global gesture_counts
    if current_image is None:
        return TriggerResponse(success=False, message="Nenhuma imagem recebida")
    
    
    frame = bridge.imgmsg_to_cv2(current_image, desired_encoding="passthrough")
    gesture = detect_hand_gesture(frame)
    if gesture in gesture_mapping:
        if gesture not in gesture_counts:
            gesture_counts[gesture] = 0
        gesture_counts[gesture] += 1

        if gesture_counts[gesture] >= 5:
            rospy.loginfo(f"Gesto {gesture} reconhecido 5 vezes seguidas")
            gesture_counts = {}  # Reset counter
            return TriggerResponse(success=True, message=f"Gesto {gesture} reconhecido 5 vezes seguidas")
        else:
            return TriggerResponse(success=True, message=f"Gesto {gesture} reconhecido {gesture_counts[gesture]} vezes")
    else:
        return TriggerResponse(success=False, message="Nenhum gesto reconhecido ou não mapeado")

def gesture_recognition_server():
    rospy.init_node('gestos_srv')
    rospy.Service('/recognize_gesture', Trigger, handle_gesture_recognition)
    rospy.Subscriber('/Imagens', Image, image_callback)
    rospy.set_param('/stop_counting', False)
    rospy.loginfo("Nó de serviço de reconhecimento de gestos iniciado.")
    rospy.spin()

if __name__ == '__main__':
    try:
        gesture_recognition_server()
    except rospy.ROSInterruptException:
        pass
