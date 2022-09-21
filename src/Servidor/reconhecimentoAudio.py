import speech_recognition as sr
import numpy as np
import sounddevice as sd
import soundfile as sf


def trataAudio(audio, taxa_amostragem, duracao):

    passa_alta = False
    amplitude = 0.3

    duracao_amostragem = int(duracao * taxa_amostragem)

    sinal_entrada = audio

    freq_corte = np.geomspace(20000, 20, sinal_entrada.shape[0])
    passaAlta_saida = np.zeros_like(sinal_entrada)
    dn_1 = 0

    for n in range(sinal_entrada.shape[0]):
        freq_quebra = freq_corte[n]

        tan = np.tan(np.pi * freq_quebra / taxa_amostragem)
        a1 = (tan - 1) / (tan + 1)

        passaAlta_saida[n] = a1 * sinal_entrada[n] + dn_1
        dn_1 = sinal_entrada[n] - a1 * passaAlta_saida[n]

    if passa_alta:
        passaAlta_saida *= -1

    audio_tratado = sinal_entrada + passaAlta_saida
    audio_tratado *= 0.5
    audio_tratado *= amplitude

    return audio_tratado


def reconhecerAudio(audio, sample_rate=44100, sample_width=2):
    rec = sr.Recognizer()

    sd.play(audio, sample_rate)
    sd.wait()

    try:
        rec.adjust_for_ambient_noise(audio)
        audiodata = sr.AudioData(audio, sample_rate, sample_width)
        texto = rec.recognize_google(audiodata, language="pt-BR", show_all=False)
    except:
        return None

    return texto


if __name__ == "__main__":
    from sys import path
    from os.path import join, dirname, realpath

    # adiciona o caminho da função de gravar áudio, feito somente para
    # reutilizar código na situação de debugging,
    # não é uma prática boa fora desse contexto!
    path.append(join(dirname(realpath(__file__)), "../Cliente"))

    from gravarAudio import gravarAudioArquivo
    import os

    gravarAudioArquivo("exemplo.wav")
    data = sf.SoundFile('exemplo.wav')
    duracao = data.frames / data.samplerate
    data, taxa_amostragem = sf.read('exemplo.wav')

    audio_tratado = trataAudio(np.array(data), taxa_amostragem, duracao)
    texto = reconhecerAudio(audio_tratado)
    print(texto)

    os.remove("exemplo.wav")
