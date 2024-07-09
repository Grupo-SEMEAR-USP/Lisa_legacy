#!/usr/bin/env python3

import rospy
import time
import pigpio
from std_msgs.msg import Int32

class ServoController:
    def __init__(self):
        # Conectar ao daemon pigpio
        self.pi = pigpio.pi()
        
        # Verificar se a conexão foi bem-sucedida
        if not self.pi.connected:
            rospy.logerr("Não foi possível conectar ao pigpio daemon.")
            exit()

        # Definir os pinos GPIO onde os servos estão conectados
        self.SERVO_PIN1 = rospy.get_param('~servo_pin1', 20)
        self.SERVO_PIN2 = rospy.get_param('~servo_pin2', 21)
        
        # Definir a largura de pulso mínima e máxima dos servos em microsegundos
        self.MIN_PULSE_WIDTH = rospy.get_param('~min_pulse_width', 500)
        self.MAX_PULSE_WIDTH = rospy.get_param('~max_pulse_width', 2500)

        # Definir o intervalo de ângulo dos servos (normalmente 0 a 180 graus)
        self.MIN_ANGLE = rospy.get_param('~min_angle', 0)
        self.MAX_ANGLE = rospy.get_param('~max_angle', 180)

        # Inicializar variáveis de controle
        self.active = False

        # Subscriber para o tópico /orelhas
        rospy.Subscriber("/orelhas", Int32, self.orelhas_callback)

        rospy.on_shutdown(self.shutdown)

    def set_servo_angle(self, pin, angle):
        # Converter o ângulo para largura de pulso
        pulse_width = self.MIN_PULSE_WIDTH + (angle - self.MIN_ANGLE) * (self.MAX_PULSE_WIDTH - self.MIN_PULSE_WIDTH) / (self.MAX_ANGLE - self.MIN_ANGLE)
        self.pi.set_servo_pulsewidth(pin, pulse_width)

    def orelhas_callback(self, msg):
        self.active = msg.data

    def start(self):
        try:
            while not rospy.is_shutdown():
                # rospy.loginfo("entrou")

                if self.active == 1:
                    rospy.loginfo("Entrou na felizinha...")
                    self.set_servo_angle(self.SERVO_PIN1, 30)
                    self.set_servo_angle(self.SERVO_PIN2, 30)
                    time.sleep(0.5)
                    self.set_servo_angle(self.SERVO_PIN1, 0)
                    self.set_servo_angle(self.SERVO_PIN2, 0)
                    time.sleep(0.5)
                    self.set_servo_angle(self.SERVO_PIN1, 30)
                    self.set_servo_angle(self.SERVO_PIN2, 30)

                if self.active == 2:  # Tristinha
                    rospy.loginfo("entrou no tristinha")
                    self.set_servo_angle(self.SERVO_PIN1, 90)
                    self.set_servo_angle(self.SERVO_PIN2, 90)
                    time.sleep(2)

                if self.active == 3:  # Bombastic
                    self.set_servo_angle(self.SERVO_PIN1, 90)
                    time.sleep(2)

                if self.active == 4:  # Ingratidão
                    self.set_servo_angle(self.SERVO_PIN1, 90)
                    self.set_servo_angle(self.SERVO_PIN2, 90)
                    time.sleep(2)

                if self.active == 5:  # Sapeca
                    self.set_servo_angle(self.SERVO_PIN1, 10)
                    self.set_servo_angle(self.SERVO_PIN2, 10)
                    time.sleep(0.5)
                    self.set_servo_angle(self.SERVO_PIN1, 0)
                    self.set_servo_angle(self.SERVO_PIN2, 0)
                    time.sleep(0.5)
                    self.set_servo_angle(self.SERVO_PIN1, 10)
                    self.set_servo_angle(self.SERVO_PIN2, 10)
                    time.sleep(0.5)

                if self.active == 6:  # Dancinha festa
                    self.set_servo_angle(self.SERVO_PIN1, 20)
                    self.set_servo_angle(self.SERVO_PIN2, 20)
                    time.sleep(0.5)
                    self.set_servo_angle(self.SERVO_PIN1, 0)
                    self.set_servo_angle(self.SERVO_PIN2, 0)
                    time.sleep(0.5)
                    self.set_servo_angle(self.SERVO_PIN1, 20)
                    self.set_servo_angle(self.SERVO_PIN2, 20)
                    time.sleep(0.5)

                if self.active == 7:  # Assustada
                    self.set_servo_angle(self.SERVO_PIN1, 30)
                    self.set_servo_angle(self.SERVO_PIN2, 30)
                    time.sleep(2)
                
                self.set_servo_angle(self.SERVO_PIN1, 0)
                self.set_servo_angle(self.SERVO_PIN2, 0)
        except rospy.ROSInterruptException:
            pass

    def shutdown(self):
        # Desligar os servos
        self.pi.set_servo_pulsewidth(self.SERVO_PIN1, 0)
        self.pi.set_servo_pulsewidth(self.SERVO_PIN2, 0)
        self.pi.stop()

if __name__ == '__main__':
    rospy.init_node('servo_node')
    controller = ServoController()
    controller.start()
    rospy.spin()
