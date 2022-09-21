import speech_recognition as sr
import numpy as np
import librosa


def trataAudio(audio):

    # Carregando o áudio
    x, sr = librosa.load(audio, sr=None)

    # Computando a magnitude e a fase do espectograma
    espectograma_completo = librosa.magphase(librosa.stft(x))

    # Aplicando uma filtragem que utiliza similaridade por cossenos
    espectograma_filtrado = librosa.decompose.nn_filter(espectograma_completo,
                                                        aggregate=np.median,
                                                        metric='cosine',
                                                        width=int(librosa.time_to_frames(2, sr=sr)))

    # A saída do filtro não deve ser maior que a entrada
    espectograma_filtrado = np.minimum(espectograma_completo, espectograma_filtrado)

    # Margens para reduzir o ruído entre a voz e máscara de segmentação
    margem_i, margem_v = 2, 10
    mascara_i = librosa.util.softmask(espectograma_filtrado,
                                      margem_i * (espectograma_completo -espectograma_filtrado),
                                      power=2)
    mascara_v = librosa.util.softmask(espectograma_completo - espectograma_filtrado,
                                      margem_v * espectograma_filtrado,
                                      power=2)

    audio_tratado = mascara_v * espectograma_completo
    audio_ruidoso = mascara_i * espectograma_completo

    return audio_tratado


def reconhecerAudio(audio, sample_rate=44100, sample_width=2):
    rec = sr.Recognizer()

    audiodata = sr.AudioData(audio, sample_rate, sample_width)

    texto = rec.recognize_google(audiodata, language="pt-BR")
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
    audio = os.getcwd() + "/exemplo.wav"

    audio_tratado = trataAudio(audio)
    texto = reconhecerAudio(audio_tratado)
    print(texto)

    arq.close()
    os.remove("exemplo.wav")
