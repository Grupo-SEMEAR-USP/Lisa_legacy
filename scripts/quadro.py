import cv2
import mediapipe as mp
import os
import numpy as np
from time import sleep

BRANCO = (255, 255, 255)
PRETO = (0, 0, 0)
AZUL = (255, 0, 0)
VERDE = (0, 255, 0)
VERMELHO = (0, 0, 255)
AZUL_CLARO = (255, 255, 0)

mp_maos = mp.solutions.hands
mp_desenho = mp.solutions.drawing_utils
maos = mp_maos.Hands()

resolucao_x = 1280
resolucao_y = 720
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, resolucao_x)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, resolucao_y)

offset = 50
contador = 0
texto = '>'
img_quadro = np.ones((resolucao_y, resolucao_x, 3), np.uint8) * 255
cor_pincel = (255, 0, 0)
espessura_pincel = 7
x_quadro, y_quadro = 0, 0

def encontra_coordenadas_maos(img, lado_invertido=False):
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    resultado = maos.process(img_rgb)
    todas_maos = []
    if resultado.multi_hand_landmarks:
        for lado_mao, marcacoes_maos in zip(resultado.multi_handedness, resultado.multi_hand_landmarks):
            info_mao = {}
            coordenadas = []
            for marcacao in marcacoes_maos.landmark:
                coord_x, coord_y, coord_z = int(marcacao.x * resolucao_x), int(marcacao.y * resolucao_y), int(marcacao.z * resolucao_x)
                coordenadas.append((coord_x, coord_y, coord_z))

            info_mao['coordenadas'] = coordenadas
            if lado_invertido:
                if lado_mao.classification[0].label == 'Left':
                    info_mao['lado'] = 'Right'
                else:
                    info_mao['lado'] = 'Left'
            else:
                info_mao['lado'] = lado_mao.classification[0].label

            todas_maos.append(info_mao)
            mp_desenho.draw_landmarks(img, marcacoes_maos, mp_maos.HAND_CONNECTIONS)

    return img, todas_maos

def dedos_levantados(mao):
    dedos = []
    for ponta_dedo in [8, 12, 16, 20]:
        if mao['coordenadas'][ponta_dedo][1] < mao['coordenadas'][ponta_dedo - 2][1]:
            dedos.append(True)
        else:
            dedos.append(False)
    return dedos

while True:
    sucesso, img = camera.read()
    img = cv2.flip(img, 1)

    img, todas_maos = encontra_coordenadas_maos(img)

    if len(todas_maos) == 1:
        info_dedos_mao1 = dedos_levantados(todas_maos[0])

        if todas_maos[0]['lado'] == 'Right' and info_dedos_mao1 == [True, False, False, True]:
            break

    if len(todas_maos) == 2:
        info_dedos_mao1 = dedos_levantados(todas_maos[0])
        info_dedos_mao2 = dedos_levantados(todas_maos[1])

        indicador_x, indicador_y, indicador_z = todas_maos[0]['coordenadas'][8]

        if sum(info_dedos_mao2) == 1:
            cor_pincel = AZUL
        elif sum(info_dedos_mao2) == 2:
            cor_pincel = VERDE
        elif sum(info_dedos_mao2) == 3:
            cor_pincel = VERMELHO
        elif sum(info_dedos_mao2) == 4:
            cor_pincel = BRANCO
        else:
            img_quadro = np.ones((resolucao_y, resolucao_x, 3), np.uint8) * 255

        espessura_pincel = int(abs(indicador_z)) // 3 + 5
        cv2.circle(img, (indicador_x, indicador_y), espessura_pincel, cor_pincel, cv2.FILLED)

        if info_dedos_mao1 == [True, False, False, False]:
            if x_quadro == 0 and y_quadro == 0:
                x_quadro, y_quadro = indicador_x, indicador_y

            cv2.line(img_quadro, (x_quadro, y_quadro), (indicador_x, indicador_y), cor_pincel, espessura_pincel)

            x_quadro, y_quadro = indicador_x, indicador_y
        else:
            x_quadro, y_quadro = 0, 0

        img = cv2.addWeighted(img, 1, img_quadro, 0.2, 0)

    cv2.imshow("Imagem", img)
    cv2.imshow('Quadro', img_quadro)
    tecla = cv2.waitKey(1)
    if tecla == 27:
        break

with open('texto.txt', 'w') as arquivo:
    arquivo.write(texto)

cv2.imwrite('quadro.png', img_quadro)
