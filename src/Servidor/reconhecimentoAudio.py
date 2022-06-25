import speech_recognition as sr
import numpy as np
from scipy.io import wavfile
from python_speech_features import mfcc, logfbank

# Função de tratamento de áudio, basicamente implementa um filtro passa-faixa
def trataAudio(audio):
    # Define a frequência da amostra e lê o sinal do arquivo de áudio
    f_sample, signal_audio = (wavfile.read(audio)) 

    # Normalizando o sinal
    pow_audio = signal_audio / np.power(2, 15)
    pow_audio = pow_audio[:100]
    time = 1000 * np.arrange(0, len(pow_audio), 1) / float(f_sample)

    signal_audio = signal_audio / np.power(2, 15)

    # Agora, trabalhando no mesmo de arquivo de entrada para aplicar a Transformada de Fourier
    signal_len = len(signal_audio)
    half_len = np.ceil((signal_len + 1) / 2.0).astype(np.int)

    # Usando a Fast Fourier transform para formar o domínio da frequência do sinal
    sinal_f = np.fft.fft(signal_audio)

    # Normalizando o domínio da frequência
    sinal_f = abs(sinal_f[0:half_len]) / signal_len
    sinal_f **= 2
    transform_len = len(sinal_f)

    # Ajustando a transformada para casos pares e ímpares 
    if signal_len % 2:
        sinal_f[1:transform_len] *= 2
    else:
        sinal_f[1:transform_len - 1] *= 2

    # Agora, serão pegos as primeiras 15000 amostras do sinal de aúdio
    signal_audio = signal_audio[:15000]

    # Usando o Coeficiente de Frequência Cepstral-Mel para extrair os recuros do sinal (extrai apenas a voz humana)
    mfcc_recursos = mfcc(signal_audio, f_sample)
    mfcc_recursos = mfcc_recursos.T

    #Gerando os recursos do banco de filtro
    fb_recursos = logfbank(signal_audio, f_sample)
    fb_recursos = fb_recursos.T

    return mfcc_recursos #TODO verificar se, de fato, são as features do MFCC que são retornadas 

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