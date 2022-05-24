import speech_recognition as sr


#TODO:  limpeza de audio


def reconhecerAudio(audio, sample_rate=44100, sample_width=2):
    rec = sr.Recognizer()

    audiodata = sr.AudioData(audio, sample_rate, sample_width)
    
    texto = rec.recognize_google(audiodata, language="pt-BR")
    return texto


if __name__ == "__main__":
    from sys import path
    from os.path import join, dirname, realpath

    #adiciona o caminho da função de gravar áudio, feito somente para
    #reutilizar código na situação de debugging,
    #não é uma prática boa fora desse contexto!
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