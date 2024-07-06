#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool
import subprocess
import os
import re
import time
import threading

current_process = None
current_gif = None  # Iniciar com None para forçar a atualização
last_state = None

educativo = 1
last_educativo = 1

# Variáveis para controle do temporizador
last_gif_time = time.time()
sleep_timeout = 300  # 5 minutos em segundos
sleep_gif_name = 'animated_sleepy.gif'
is_sleeping = False  # Novo estado para verificar se está no modo sleepy

def play_gif(gif_name):
    global current_process, current_gif, last_gif_time
    try:
        # Define o caminho absoluto para o diretório das imagens
        gif_path = os.path.join('/home/lisa/Lisa/lisa_ws/src/estrutura/Images/telas', gif_name)
        rospy.loginfo(f"Tentando exibir GIF: {gif_path}")

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
            rospy.loginfo("Terminando processo anterior de exibição de GIF.")
            current_process.terminate()
            current_process.wait()  # Espera o término completo do processo

        # Sempre executa o comando para garantir que o GIF seja exibido
        rospy.loginfo(f"Executando comando: {' '.join(command)}")
        current_process = subprocess.Popen(command)
        current_gif = gif_name
        last_gif_time = time.time()  # Atualiza o tempo da última execução de GIF
    except Exception as e:
        rospy.logerr(f"Erro ao tentar exibir o GIF: {e}")

def estado_callback(data):
    global last_state, educativo, last_educativo, is_sleeping
    if data.data != last_state:
        last_state = data.data
        if data.data:
            rospy.loginfo("Estado mudou para True. Exibindo animated_standard_cursor.gif.")
            play_gif('animated_movimento.gif')
            time.sleep(2)
            play_gif('animated_standard_cursor.gif')
            is_sleeping = False  # Reset sleeping state
        else:
            rospy.loginfo("Estado mudou para False. Exibindo animated_standard.gif.")
            play_gif('animated_standard.gif')
            last_gif_time = time.time()  # Reset the inactivity timer

def modo_amor():
    rospy.loginfo("Modo Amor Ativado!")
    play_gif('animated_inlove.gif')
    time.sleep(10)
    play_gif('animated_standard.gif')

def modo_festa():
    rospy.loginfo("Modo Festa Ativado!")
    play_gif('animated_party.gif')
    time.sleep(5)
    play_gif('animated_dizzy.gif')
    time.sleep(2)
    play_gif('animated_standard.gif')

def modo_educativo():
    global educativo, last_educativo
    rospy.loginfo("Modo Educativo Ativado!")
    play_gif(f'LISA - MODO EDUCATIVO.gif')
    play_gif(f'animated_educativo_{educativo}.gif')
    if educativo == 1:
        time.sleep(28)
    else:
        time.sleep(31)
    educativo += 1
    if educativo > 3:
        educativo = 1
    play_gif('animated_standard.gif')

def modo_susto():
    rospy.loginfo("Modo Susto Ativado!")
    play_gif('animated_scared.gif')
    time.sleep(10)
    play_gif('animated_standard.gif')

def resultados_callback(data):
    if "ativando o modo amor" in data.data:
        modo_amor()
    elif "ativando o modo festa" in data.data:
        modo_festa()
    elif "ativando o modo educativo" in data.data:
        modo_educativo()
    elif "ativando o modo susto" in data.data:
        modo_susto()
    else:
        gesture_mapping = {
            'Thumb_Up': 'grooving', # feliz
            'Thumb_Down': 'sad', # triste
            'Closed_Fist': 'angry', # ingratidão
            'Victory': 'star', # olho estrela
            'Pointing_Up': 'naughty', # sapequinha
            'Open_Palm': 'bombastic', # The rock
        }

        match = re.search(r'Gesto reconhecido: Gesto (\w+) reconhecido 3 vezes seguidas', data.data)
        if match:
            gesture_name = match.group(1)
            if gesture_name in gesture_mapping:
                rospy.loginfo(f"Gesto reconhecido: {gesture_name}. Exibindo animated_{gesture_mapping[gesture_name]}.gif.")
                play_gif(f'animated_{gesture_mapping[gesture_name]}.gif')
            else:
                rospy.logwarn(f"Gesto '{gesture_name}' não encontrado no mapeamento.")
        else:
            rospy.logwarn("Formato da mensagem não reconhecido.")

def sleep_timer():
    global last_gif_time, sleep_timeout, sleep_gif_name, is_sleeping
    while not rospy.is_shutdown():
        current_time = time.time()
        if current_time - last_gif_time > sleep_timeout and not is_sleeping:
            rospy.loginfo("Tempo de inatividade excedido. Exibindo animated_sleepy.gif.")
            play_gif(sleep_gif_name)
            is_sleeping = True  # Indica que o modo sleepy foi ativado
        time.sleep(1)  # Verifica a cada segundo

def gif_display_node():
    rospy.init_node('gif_display_node', anonymous=True)
    rospy.Subscriber('/estado', Bool, estado_callback)
    rospy.Subscriber('/resultados', String, resultados_callback)

    # Exibe o GIF padrão ao iniciar
    rospy.loginfo("Exibindo GIF padrão 'animated_standard.gif'.")
    play_gif('animated_standard.gif')

    # Inicia o temporizador de sono em uma thread separada
    sleep_thread = threading.Thread(target=sleep_timer)
    sleep_thread.daemon = True
    sleep_thread.start()

    rospy.spin()

if __name__ == '__main__':
    try:
        gif_display_node()
    except rospy.ROSInterruptException:
        pass
