import speech_recognition as sr


#TODO:  limpeza de audio


def reconhecerAudio(audio, sample_rate=44100, sample_width=2):
    recognizer = sr.Recognizer()

    arq = sr.AudioData(audio, sample_rate, sample_width)
    
    texto = recognizer.recognize_google(arq, language="pt-BR")
    return texto


if __name__ == "__main__":
    from sys import path
    from os.path import join, dirname, realpath
    path.append(join(dirname(realpath(__file__)), "../Cliente"))

    from gravarAudio import gravarAudioArquivo
    import os

    gravarAudioArquivo("exemplo.wav")
    arq = open("exemplo.wav", "rb")
    audio = arq.read()

    texto = reconhecerAudio(audio)
    print(texto)

    arq.close()
    os.remove("exemplo.wav")