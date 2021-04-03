# -*- coding: UTF-8 -*-
#!/usr/bin/env python3

# NOTE: this example requires PyAudio because it uses the Microphone class
import speech_recognition as sr
from unidecode import unidecode
import pyaudio
import string
import nltk
import cv2

def speech_text():
	# obtain audio from the microphone
	stopword = nltk.corpus.stopwords.words('portuguese')
	mic = sr.Recognizer()
	mic.pause_threshold = 1
	with sr.Microphone() as source:
		print("\nDetectando ruido ambiente...")
		mic.adjust_for_ambient_noise(source,duration=10)
		print("Fale!")
		audio = mic.listen(source)

		print("Ok!")

	# recognize speech using Google Speech Recognition
		try:
			txt = mic.recognize_google(audio, language='pt-BR')
			
			txt = unidecode(txt)
			txt = txt.lower()
			print("Voce disse: " + txt)
			
			txt = [txt]
			txt = [pt for pt in txt if pt not in string.punctuation]
			txt = txt[0]
			print(txt)

			lista_pre = [pt for pt in txt.split() if pt not in stopword]
			print(lista_pre)

			lematizado = []
			stemmer = nltk.stem.RSLPStemmer()
			for word in lista_pre:
				lematizado.append(stemmer.stem(word))
			print(lematizado)

		except sr.UnknownValueError:
			print("Entendi nn")

cv2.namedWindow("hello")
key = cv2.waitKey(10)

while(True):
	key = cv2.waitKey(1)
	if key != -1: # Loop until a key has been pressed
		speech_text()