from gtts import gTTS


#TODO:  o audio sai como .mp3, temos que converter esse audio para wav para a Lisa
#       melhorar o TTS com librosa


def gerarStreamAudio(texto):
    tts = gTTS(texto, lang="pt")

    return tts.stream


if __name__ == "__main__":
    import playsound
    import os

    gen_audio = gerarStreamAudio("Oi! Eu sou a Lisa")
    
    f = open("exemplo.mp3", "wb")
    for s in gen_audio():
        f.write(s)
    f.close()

    playsound.playsound("exemplo.mp3")
    os.remove("exemplo.mp3")
