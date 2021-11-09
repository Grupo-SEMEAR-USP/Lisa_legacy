import os
import time
import playsound
import speech_recognition as sr
from gtts import gTTS


def speak(text):
	tts = gTTS(text=text, lang="pt")
	filename = "voice.mp3"
	tts.save(filename)
	playsound.playsound(filename)

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


text = get_audio().lower()

if "segredo da vida" in text or "pop" in text:
    speak("Ah! O segredo da vida é")
    playsound.playsound('C:\\Users\\breno\\Downloads\\never.mp3')