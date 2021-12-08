import os
import time
import speech_recognition as sr
import pyaudio

import sintese_voz

def get_audio():
	r = sr.Recognizer()
	with sr.Microphone() as source:
		audio = r.listen(source)
		said = ""

		try:
			said = r.recognize_google(audio, language = 'pt-BR')
			print(said)
		except Exception as e:
			print("Exception " + str(e))
	return said

if __name__ == '__main__':
	text = get_audio().lower()
	sintese_voz.speak(text)
	print(text)