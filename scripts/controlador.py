#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import String, Bool
import time
import pigpio

class Controlador:
    def __init__(self):
        rospy.init_node('controlador_node')
        rospy.wait_for_service('/get_finger_count')
        rospy.wait_for_service('/recognize_gesture')
        rospy.wait_for_service('/recognize_face')

        self.get_finger_count = rospy.ServiceProxy('/get_finger_count', Trigger)
        self.recognize_gesture = rospy.ServiceProxy('/recognize_gesture', Trigger)
        self.recognize_face = rospy.ServiceProxy('/recognize_face', Trigger)
        self.image_sub = rospy.Subscriber('/Imagens', Image, self.image_callback)
        self.result_pub = rospy.Publisher('/resultados', String, queue_size=10)
        self.state_pub = rospy.Publisher('/estado', Bool, queue_size=10)
        self.current_image = None
        self.stop_counting = False
        self.gesture_active = False
        self.face_active = False
        self.rate = rospy.Rate(1)  # 1 Hz

    #     # Inicializa o pigpio e configura o servo
    #     self.pi = pigpio.pi()
    #     self.servo_pin = 18
    #     self.servo_min = 500  # 0 graus
    #     self.servo_max = 2500  # 180 graus

    # def __del__(self):
    #     # Libera os recursos do pigpio ao destruir a instância do controlador
    #     self.pi.set_servo_pulsewidth(self.servo_pin, 0)
    #     self.pi.stop()

    # def set_servo_angle(self, angle):
    #     pulsewidth = self.servo_min + (angle / 180.0) * (self.servo_max - self.servo_min)
    #     self.pi.set_servo_pulsewidth(self.servo_pin, pulsewidth)

    def image_callback(self, msg):
        self.current_image = msg

    def run(self):
        while not rospy.is_shutdown():
            if self.current_image is None:
                rospy.loginfo("Aguardando imagem...")
                self.rate.sleep()
                continue
            
            if self.stop_counting:
                if self.gesture_active:
                    rospy.loginfo("Reconhecimento de gestos em andamento...")
                    self.state_pub.publish(True) # Booleana para ver o estado
                    try:
                        gesture_response = self.recognize_gesture()
                        if gesture_response.success:
                            rospy.loginfo(f"Gesto reconhecido: {gesture_response.message}")
                            if "3 vezes seguidas" in gesture_response.message:
                                rospy.loginfo("Gesto reconhecido 3 vezes. Reativando contador de dedos.")
                                self.result_pub.publish(f"Gesto reconhecido: {gesture_response.message}")
                                self.gesture_active = False
                                self.stop_counting = False
                                rospy.set_param('/stop_counting', False)
                        else:
                            rospy.loginfo("Nenhum gesto reconhecido ou reconhecimento não concluído.")
                    except rospy.ServiceException as e:
                        rospy.logerr("Falha ao chamar o serviço: %s", e)
                    self.rate.sleep()
                    continue
                
                if self.face_active:
                    rospy.loginfo("Reconhecimento de rostos em andamento...")
                    try:
                        face_response = self.recognize_face()
                        if face_response.success:
                            rospy.loginfo(f"Rosto reconhecido: {face_response.message}")
                            self.result_pub.publish(f"Rosto reconhecido: {face_response.message}")
                            self.face_active = False
                            self.stop_counting = False
                            rospy.set_param('/stop_counting', False)
                        else:
                            rospy.loginfo("Nenhum rosto reconhecido ou reconhecimento não concluído.")
                    except rospy.ServiceException as e:
                        rospy.logerr("Falha ao chamar o serviço: %s", e)
                    self.rate.sleep()
                    continue
                
            try:
                response = self.get_finger_count()
                if response.success:
                    finger_count = int(response.message)
                    if finger_count == 1:
                        rospy.loginfo("Contador é 1, ativando modo amor")
                        self.result_pub.publish("ativando o modo amor")
                        self.stop_counting = True
                        time.sleep(7)
                        self.stop_counting = True
                    elif finger_count == 2:
                        rospy.loginfo("Contador é 2, ativando serviço de gestos.")
                        self.gesture_active = True
                        self.stop_counting = True
                        rospy.set_param('/stop_counting', True)
                    elif finger_count == 3:
                        rospy.loginfo("Contador é 3, ativando serviço de reconhecimento de rostos.")
                        self.face_active = True
                        self.stop_counting = True
                        rospy.set_param('/stop_counting', True)
                    elif finger_count == 4:
                        rospy.loginfo("Contador é 4, ativando modo festa.")
                        self.result_pub.publish("ativando o modo festa")
                        rospy.sleep(5) #Pausa de 5 segundos
                        self.stop_counting = True
                        # for _ in range(4):
                        #     rospy.loginfo("Controlando Servos")
                        #     self.set_servo_angle(0)
                        #     time.sleep(0.5)
                        #     self.set_servo_angle(90)
                        #     time.sleep(0.5)
                        # self.stop_counting = True
                        time.sleep(7)
                    elif finger_count == 5:
                        rospy.loginfo("Contador é 5, ativando modo educativo.")
                        self.result_pub.publish("ativando o modo educativo")
                        self.stop_counting = True
                        time.sleep(7)
                        self.stop_counting = True
                    elif finger_count == 10:
                        rospy.loginfo("Contador é 10, ativando modo susto.")
                        self.result_pub.publish("ativando o modo susto")
                        self.stop_counting = True
                        time.sleep(7)
                        self.stop_counting = True
                    else:
                        self.result_pub.publish(f"Numero de dedos: {finger_count}")
                else:
                    rospy.loginfo("Falha ao obter contagem de dedos para comandos.")
            except rospy.ServiceException as e:
                rospy.logerr("Falha ao chamar o serviço: %s", e)
            
            self.rate.sleep()
            self.state_pub.publish(False) # Booleana para ver o estado

if __name__ == '__main__':
    try:
        controlador = Controlador()
        controlador.run()
    except rospy.ROSInterruptException:
        pass
