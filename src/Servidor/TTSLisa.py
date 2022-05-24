from gtts import gTTS


#TODO:  o audio sai como .mp3, temos que converter esse audio para wav para a Lisa
#       melhorar o TTS com librosa


def gerarStreamAudio(texto):
    tts = gTTS(texto, lang="pt")

    return tts.stream


if __name__ == "__main__":
    import playsound
    import os

    gerador = gerarStreamAudio("Oi! Eu sou a Lisa")
    
    arq = open("exemplo.mp3", "wb")
    for s in gerador():
        arq.write(s)
    arq.close()

    playsound.playsound("exemplo.mp3")
    os.remove("exemplo.mp3")
