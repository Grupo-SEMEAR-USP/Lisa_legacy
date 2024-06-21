#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import subprocess
import os
import re

# Mapeamento de gestos para nomes de arquivos GIF
gesture_mapping = {
    'Thumb_Up': 'FELIZINHA',
    'Thumb_Down': '(IN)GRATIDAO',
    'Open_Palm': 'TONTURA',
    'Closed_Fist': 'TRISTE',
    'Victory': 'SAPECA',
    'Pointing_Up': 'UWU',
}

timeout_duration = rospy.Duration(10)  # Duração do timeout em segundos
last_message_time = None
current_process = None
background_process = None

def play_gif_with_mpv(gesture):
    global current_process
    try:
        # Obter o diretório do script atual e ir para o diretório pai
        current_directory = os.path.dirname(os.path.abspath(__file__))
        parent_directory = os.path.dirname(current_directory)

        # Construir o caminho relativo para o arquivo GIF
        gif_path = os.path.join(parent_directory, 'Images', 'telas', f'LISA - {gesture_mapping[gesture]}.gif')

        # Verifica se o arquivo existe
        if not os.path.exists(gif_path):
            rospy.logerr(f"O arquivo GIF não foi encontrado: {gif_path}")
            return

        # Certifique-se de que a variável DISPLAY está configurada para :0
        os.environ['DISPLAY'] = ':0'

        # Comando para executar o mpv com um método de saída de vídeo específico
        command = ['mpv', '--fullscreen', '--loop=inf', '--vo=x11', gif_path]

        # Termina o processo anterior, se houver
        if current_process:
            current_process.terminate()
            current_process.wait()  # Espera o término completo do processo

        # Executa o comando
        current_process = subprocess.Popen(command)
    except Exception as e:
        rospy.logerr(f"Erro ao tentar exibir o GIF: {e}")

def play_background_gif():
    global background_process
    try:
        # Obter o diretório do script atual e ir para o diretório pai
        current_directory = os.path.dirname(os.path.abspath(__file__))
        parent_directory = os.path.dirname(current_directory)

        # Construir o caminho relativo para o arquivo GIF padrão
        gif_path = os.path.join(parent_directory, 'Images', 'telas', 'LISA - PISCAR SIMPLES.gif')

        # Verifica se o arquivo existe
        if not os.path.exists(gif_path):
            rospy.logerr(f"O arquivo GIF padrão não foi encontrado: {gif_path}")
            return

        # Certifique-se de que a variável DISPLAY está configurada para :0
        os.environ['DISPLAY'] = ':0'

        # Comando para executar o mpv com um método de saída de vídeo específico
        command = ['mpv', '--fullscreen', '--loop=inf', '--vo=x11', gif_path]

        # Executa o comando apenas se o processo de fundo não estiver em execução
        if not background_process:
            background_process = subprocess.Popen(command)
    except Exception as e:
        rospy.logerr(f"Erro ao tentar exibir o GIF de fundo: {e}")

def callback(data):
    global last_message_time
    rospy.loginfo(f"Recebido: {data.data}")

    # Atualizar o tempo da última mensagem recebida
    last_message_time = rospy.Time.now()

    # Usar regex para extrair o nome do gesto
    match = re.search(r'Gesto reconhecido: Gesto (\w+) reconhecido 3 vezes seguidas', data.data)
    if match:
        gesture_name = match.group(1)
        if gesture_name in gesture_mapping:
            play_gif_with_mpv(gesture_name)
        else:
            rospy.logwarn(f"Gesto '{gesture_name}' não encontrado no mapeamento.")
    else:
        rospy.logwarn("Formato da mensagem não reconhecido.")

def check_timeout(event):
    global last_message_time, current_process, background_process
    if last_message_time and (rospy.Time.now() - last_message_time > timeout_duration):
        rospy.loginfo("Tópico 'resultados' não atualizado por muito tempo. Voltando ao GIF de fundo...")
        if current_process:
            current_process.terminate()
            current_process = None
        if not background_process:
            play_background_gif()

def listener():
    global last_message_time
    rospy.init_node('gif_display_node', anonymous=True)
    last_message_time = rospy.Time.now()
    rospy.Subscriber('/resultados', String, callback)
    rospy.Timer(rospy.Duration(1), check_timeout)  # Verifica o timeout a cada segundo
    play_background_gif()  # Começa com o GIF de fundo
    rospy.spin()

if __name__ == '__main__':
    listener()
