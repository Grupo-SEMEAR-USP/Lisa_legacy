#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import String, Bool, Int32
import time

class Controlador:
    def __init__(self):
        rospy.init_node('controlador_node')
        rospy.wait_for_service('/get_finger_count')
        rospy.wait_for_service('/recognize_gesture')

        self.get_finger_count = rospy.ServiceProxy('/get_finger_count', Trigger)
        self.recognize_gesture = rospy.ServiceProxy('/recognize_gesture', Trigger)
        self.face_detected_sub = rospy.Subscriber('/face_detected', Bool, self.face_detected_callback)
        self.image_sub = rospy.Subscriber('/Imagens', Image, self.image_callback)
        self.result_pub = rospy.Publisher('/resultados', String, queue_size=10)
        self.state_pub = rospy.Publisher('/estado', Bool, queue_size=10)
        self.orelhas_pub = rospy.Publisher('/orelhas', Int32, queue_size=10)
        self.current_image = None
        self.face_detected = False
        self.stop_counting = False
        self.gesture_active = False
        self.mode_active = False
        self.drawing_active = False  # Variável para controlar o serviço de desenho
        self.rate = rospy.Rate(1)  # 1 Hz

    def image_callback(self, msg):
        self.current_image = msg
    def face_detected_callback(self, msg):
        self.face_detected = msg.data
        if not self.face_detected:
            self.stop_counting = not msg.data
            rospy.set_param('/stop_counting', self.stop_counting)
        
    def reset_states(self):
        self.gesture_active = False
        self.mode_active = False
        self.stop_counting = False
        self.drawing_active = False  # Reinicia o estado do serviço de desenho
        rospy.set_param('/stop_counting', False)

    def run(self):
        while not rospy.is_shutdown():
            if self.current_image is None:
                rospy.loginfo("Aguardando imagem...")
                self.rate.sleep()
                continue

            if self.stop_counting:
                if self.gesture_active:
                    rospy.loginfo("Reconhecimento de gestos em andamento...")
                    self.state_pub.publish(True)  # Booleana para ver o estado
                    try:
                        gesture_response = self.recognize_gesture()
                        if gesture_response.success:
                            rospy.loginfo(f"Gesto reconhecido: {gesture_response.message}")
                            if "5 vezes seguidas" in gesture_response.message:
                                rospy.loginfo("Gesto reconhecido 5 vezes. Reativando contador de dedos.")
                                self.result_pub.publish(f"Gesto reconhecido: {gesture_response.message}")
                                time.sleep(7)
                                self.rate.sleep()
                                self.reset_states()
                        else:
                            rospy.loginfo("Nenhum gesto reconhecido ou reconhecimento não concluído.")
                    except rospy.ServiceException as e:
                        rospy.logerr("Falha ao chamar o serviço: %s", e)
                    self.rate.sleep()
                    continue


                if self.drawing_active:
                    rospy.loginfo("Serviço de desenho no quadro ativado...")
                    try:
                        draw_response = self.draw_on_board()
                        if draw_response.success:
                            rospy.loginfo("Desenho no quadro concluído.")
                        else:
                            rospy.loginfo("Falha ao desenhar no quadro.")
                    except rospy.ServiceException as e:
                        rospy.logerr("Falha ao chamar o serviço de desenho: %s", e)
                    self.rate.sleep()
                    continue

                else:
                    rospy.loginfo("Não está contando e nenhum serviço ativo...")
                    if self.face_detected:
                        self.reset_states()
                    continue

            if not self.stop_counting:
                try:
                    response = self.get_finger_count()
                    if response.success:
                        finger_count = int(response.message)

                        if finger_count == 1 and not self.mode_active:
                            rospy.loginfo("Contador é 1, ativando modo amor")
                            self.result_pub.publish("ativando o modo amor")
                            self.mode_active = True
                            self.stop_counting = True
                            rospy.set_param('/stop_counting', True)
                            self.orelhas_pub.publish(1)  # Publica 1 no tópico /orelhas
                            time.sleep(7)
                            self.orelhas_pub.publish(0)  # Publica 1 no tópico /orelhas
                            self.reset_states()
                            self.rate.sleep()  # Adiciona uma pequena pausa para evitar reiniciar imediatamente

                        elif finger_count == 2 and not self.mode_active:
                            rospy.loginfo("Contador é 2, ativando serviço de gestos.")
                            self.gesture_active = True
                            self.stop_counting = True
                            rospy.set_param('/stop_counting', True)

                        elif finger_count == 4 and not self.mode_active:
                            rospy.loginfo("Contador é 4, ativando modo festa.")
                            self.result_pub.publish("ativando o modo festa")
                            self.mode_active = True
                            rospy.sleep(5)  # Pausa de 5 segundos
                            self.stop_counting = True
                            rospy.set_param('/stop_counting', True)
                            time.sleep(7)
                            self.reset_states()
                            self.rate.sleep()  # Adiciona uma pequena pausa para evitar reiniciar imediatamente

                        elif finger_count == 5 and not self.mode_active:
                            rospy.loginfo("Contador é 5, ativando modo educativo.")
                            self.result_pub.publish("ativando o modo educativo")
                            self.mode_active = True
                            self.stop_counting = True
                            rospy.set_param('/stop_counting', True)
                            time.sleep(31)
                            self.rate.sleep() 
                            self.reset_states() 
                            
                        # elif finger_count == 6 and not self.mode_active:
                        #     rospy.loginfo("Contador é 7, ativando serviço de desenho no quadro.")
                        #     self.drawing_active = True
                        #     self.stop_counting = True
                        #     rospy.set_param('/stop_counting', True)

                        elif finger_count == 10 and not self.mode_active:
                            rospy.loginfo("Contador é 10, ativando modo susto.")
                            self.result_pub.publish("ativando o modo susto")
                            self.mode_active = True
                            self.stop_counting = True
                            rospy.set_param('/stop_counting', True)
                            time.sleep(7)
                            self.rate.sleep()  
                            self.reset_states() # Adiciona uma pequena pausa para evitar reiniciar imediatamente
                            
                        elif finger_count == 11 and not self.mode_active:
                            rospy.loginfo("Contador identificou o dedo do meio")
                            self.result_pub.publish("ativando o modo raiva")
                            self.mode_active = True
                            self.stop_counting = True
                            rospy.set_param('/stop_counting', True)
                            time.sleep(7)
                            self.rate.sleep()
                            self.reset_states() # Adiciona uma pequena pausa para evitar reiniciar imediatamente

                        else:
                            self.result_pub.publish(f"Numero de dedos: {finger_count}")
                    else:
                        rospy.loginfo("Falha ao obter contagem de dedos para comandos.")
                except rospy.ServiceException as e:
                    rospy.logerr("Falha ao chamar o serviço: %s", e)

            self.rate.sleep()
            self.state_pub.publish(False)  # Booleana para ver o estado

if __name__ == '__main__':
    try:
        controlador = Controlador()
        controlador.run()
    except rospy.ROSInterruptException:
        pass
