import numpy as np
import cv2

cap = cv2.VideoCapture(0) #captura o video da webcam
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml') #classificador já pré treinado que eciste na biblioteca do open cv que identifica rostos
eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_eye.xml') #classificador já pré treinado que eciste na biblioteca do open cv que identifica olhos

while True:
    ret, frame = cap.read() #forma que nossa imagem vai ser apresentada(frame)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #convertendo a imagem para escala de cinza
    faces = face_cascade.detectMultiScale(gray, 1.3, 5) #retorna a localização das faces que aparecem na imagem

    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 5) #para desenhar um retangulo azul em volta do rosto encontrado
        
        #obterá a localizao do rosto identificado com a imagem cinza e colorida
        roi_gray = gray[y:y+w, x:x+w] 
        roi_color = frame[y:y+h, x:x+w]

        eyes = eye_cascade.detectMultiScale(roi_gray, 1.3, 5) #detecta os olhos que aparecem na imagem
        for (ex, ey, ew, eh) in eyes:
            cv2.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0), 5) #desenhar um retangulo verde em volta dos olhos

    cv2.imshow('frame', frame) #apresenta a imagem na tela

    if cv2.waitKey(1) == ord('q'): #se clicar em 'q' fecha a janela
        break

cap.release() #libera o recurso da camera
cv2.destroyAllWindows() #desativando a webcam