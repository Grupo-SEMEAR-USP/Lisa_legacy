import speech_recognition as sr
import numpy as np
from scipy.io import wavfile
from python_speech_features import mfcc, logfbank
from io import BytesIO

def trataAudio(audio):
    ''' 
    A função trataAudio retorna o áudio de entrada com uma 
    filtragem específica para voz humana,
    resumindo é feito um filtro passa-faixa 
    '''

    # Define a frequência da amostra e lê o sinal do arquivo de áudio
    freq_amostra, sinal_audio = (wavfile.read(BytesIO(audio))) 

    # Normalizando o sinal
    pow_audio = sinal_audio / np.power(2, 15)
    pow_audio = pow_audio[:100]

    sinal_audio = sinal_audio / np.power(2, 15)

    # Agora, trabalhando no mesmo de arquivo de entrada para aplicar a Transformada de Fourier
    sinal_tam = len(sinal_audio)
    metade_tam = np.ceil((sinal_tam + 1) / 2.0).astype(int)

    # Usando a Fast Fourier transform para formar o domínio da frequência do sinal
    sinal_f = np.fft.fft(sinal_audio)

    # Normalizando o domínio da frequência
    sinal_f = abs(sinal_f[0:metade_tam]) / sinal_tam
    sinal_f **= 2
    transformada_tam = len(sinal_f)

    # Ajustando a transformada para casos pares e ímpares 
    if sinal_tam % 2:
        sinal_f[1:transformada_tam] *= 2
    else:
        sinal_f[1:transformada_tam - 1] *= 2

    # Agora, serão pegos as primeiras 15000 amostras do sinal de aúdio
    sinal_audio = sinal_audio[:15000]

    # Usando o Coeficiente de Frequência Cepstral-Mel para extrair os recuros do sinal (extrai apenas a voz humana)
    mfcc_recursos = mfcc(sinal_audio, freq_amostra)
    mfcc_recursos = mfcc_recursos.T

    #Gerando os recursos do banco de filtro
    fb_recursos = logfbank(sinal_audio, freq_amostra)
    fb_recursos = fb_recursos.T

    # Retorna a distribuição do banco de filtos no qual fica evidente a disposição das baixas e altas frequências
    return fb_recursos 

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

    # audio_tratado = trataAudio(audio)
    texto = reconhecerAudio(audio)
    print(texto)

    arq.close()
    os.remove("exemplo.wav")