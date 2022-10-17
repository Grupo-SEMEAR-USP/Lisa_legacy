import speech_recognition as sr
import structlog

logger = structlog.get_logger("recAudio")

def reconhecerAudio(audio, sample_rate=44100, sample_width=2):
    try:
        rec = sr.Recognizer()
        audiodata = sr.AudioData(audio, sample_rate, sample_width)

        return rec.recognize_google(audiodata, language="pt-BR")
    except sr.UnknownValueError:
        logger.warning("Nada foi reconhecido em um áudio, retornando vazio")
        return ""
    except sr.RequestError:
        logger.error("Erro na request de reconhecimento")
        raise ConnectionError()


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