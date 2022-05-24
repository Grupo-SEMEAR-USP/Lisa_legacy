import requests
import playsound
import os
from gravarAudio import gravarAudioArquivo


#TODO:  criar um cliente mais organizado, 
#       que se possa escolher o que se quer fazer


response = requests.post("http://localhost:8080/registrar")
uid = response.headers.get("uid")
print(uid)


response = requests.post("http://localhost:8080/para_audio", 
    "Oi! Eu sou a Lisa", headers={"content-type": "text/plain"})

f = open("exemplo.mp3", "wb")
f.write(response.content)
f.close()
playsound.playsound("exemplo.mp3")
os.remove("exemplo.mp3")


gravarAudioArquivo("exemplo.wav")
f = open("exemplo.wav", "rb")
response = requests.post("http://localhost:8080/para_texto",
    f, headers={"content-type": "audio/wav"})

print(response.content)
f.close()
os.remove("exemplo.wav")


response = requests.get(f"http://localhost:8080/responder/{uid}",
    "Oi Lisa", headers={"content-type": "text/plain"})

f = open("exemplo.mp3", "wb")
f.write(response.content)
f.close()
playsound.playsound("exemplo.mp3")
os.remove("exemplo.mp3")
