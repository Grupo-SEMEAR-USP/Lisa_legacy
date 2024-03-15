#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

# O topico precisar ser igual ao topico do publicador
nodeName, topicName = 'Leitor_de_Messagem', 'Informacao'

# Função de Resposta
# Toda vez que uma messagem chegar ao nó leitor (subscriber)m a função será executada
# Ela printa uma mensagem, Recebemos a mensagem XX (número)
def callbackFunction(message):
    print("Recemos a mensagem %d" %message.data)

rospy.init_node(nodeName)

rospy.subscriber(topicName, Int32, callbackFunction)

# É uma como uma loop while, basicamente não retorna enquanto o nó receber mensagem
rospy.spin()
