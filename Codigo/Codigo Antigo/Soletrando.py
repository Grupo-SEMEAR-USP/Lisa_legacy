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
soletrar = text + ","

for letter in text:
    if letter == "a":
        soletrar = soletrar + ("A, ")
    if letter == "b":
        soletrar = soletrar + ("Bê, ")
    if letter == "c":
        soletrar = soletrar + ("Cê, ")
    if letter == "d":
        soletrar = soletrar + ("Dê, ")
    if letter == "e":
        soletrar = soletrar + ("Ê, ")
    if letter == "f":
        soletrar = soletrar + ("É fi, ")
    if letter == "g":
        soletrar = soletrar + ("Gê, ")
    if letter == "h":
        soletrar = soletrar + ("A gá, ")
    if letter == "i":
        soletrar = soletrar + ("I, ")
    if letter == "j":
        soletrar = soletrar + ("Jota, ")
    if letter == "k":
        soletrar = soletrar + ("Ká, ")
    if letter == "l":
        soletrar = soletrar + ("É li, ")
    if letter == "m":
        soletrar = soletrar + ("Ê mi, ")
    if letter == "n":
        soletrar = soletrar + ("Ê ni, ")
    if letter == "o":
        soletrar = soletrar + ("Ô, ")
    if letter == "p":
        soletrar = soletrar + ("Pê, ")
    if letter == "q":
        soletrar = soletrar + ("Quê, ")
    if letter == "r":
        soletrar = soletrar + ("É ri, ")
    if letter == "s":
        soletrar = soletrar + ("É si, ")
    if letter == "t":
        soletrar = soletrar + ("Tê, ")
    if letter == "u":
        soletrar = soletrar + ("Ú, ")
    if letter == "v":
        soletrar = soletrar + ("Vê, ")
    if letter == "w":
        soletrar = soletrar + ("Dábliu, ")
    if letter == "x":
        soletrar = soletrar + ("Xis, ")
    if letter == "y":
        soletrar = soletrar + ("Ípsilom, ")
    if letter == "z":
        soletrar = soletrar + ("Zê, ")
    if letter == "ç":
        soletrar = soletrar + ("Cê cedilha, ")
    if letter == "á":
        soletrar = soletrar + ("A com acento agudo, ")
    if letter == "ã":
        soletrar = soletrar + ("A com til, ")
    if letter == "â":
        soletrar = soletrar + ("A com acento circunflexo, ")
    if letter == "à":
        soletrar = soletrar + ("A com crase, ")
    if letter == "ó":
        soletrar = soletrar + ("Ô com acento agudo, ")
    if letter == "õ":
        soletrar = soletrar + ("Ô com til, ")
    if letter == "ô":
        soletrar = soletrar + ("Ô com acento circunflexo, ")
    if letter == "é":
        soletrar = soletrar + ("Ê com acento agudo, ")
    if letter == "ê":
        soletrar = soletrar + ("Ê com acento circunflexo, ")
    if letter == "ú":
        soletrar = soletrar + ("Ê com acento agudo, ")
    if letter == "ü":
        soletrar = soletrar + ("Ú com trema, ")
    if letter == "í":
        soletrar = soletrar + ("I com acento agudo, ")
    if letter == "0":
        soletrar = soletrar + ("Zero, ")
    if letter == "1":
        soletrar = soletrar + ("Um, ")
    if letter == "2":
        soletrar = soletrar + ("Dois, ")
    if letter == "3":
        soletrar = soletrar + ("Três, ")
    if letter == "4":
        soletrar = soletrar + ("Quatro, ")
    if letter == "5":
        soletrar = soletrar + ("Cinco, ")
    if letter == "6":
        soletrar = soletrar + ("Seis, ")
    if letter == "7":
        soletrar = soletrar + ("Sete, ")
    if letter == "8":
        soletrar = soletrar + ("Oito, ")
    if letter == "9":
        soletrar = soletrar + ("Nove, ")

    
speak(soletrar)
    

#if "olá" in text or "oi" in text:
#    speak("olá, como vai?")
#if "qual é o seu nome" in text:
#    speak("Ainda não tenho um nome, mas espero ganhar um em breve")
#if "como vai" in text:
#    speak("Estou bem, e você?")
#if "quem é você" in text:
#    speak("Por enquanto, sou um programa em python que reconhece falas e responde baseado nelas")
