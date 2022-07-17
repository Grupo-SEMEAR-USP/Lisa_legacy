import gtts
import pydub
import pyrubberband as pyrb
import numpy as np
import io


def gerarAudio(texto):
    '''
    A função gerarAudio cria um áudio de Text to Speech a partir de um texto,
    sendo baseado no text to speech do google, mas com uma mudança de voz para
    ser mais amigável e menos cansativo de ouvir. O áudio de retorno é um array
    de numpy contendo todo o áudio descomprimido.
    '''
    tts = gtts.gTTS(texto, lang="pt")

    dados_mp3 = io.BytesIO()
    tts.write_to_fp(dados_mp3)
    dados_mp3.seek(0)

    audio = pydub.AudioSegment.from_mp3(dados_mp3)
    audio.set_channels(1)

    dados_wav = np.array(audio.get_array_of_samples())

    dados_melhor = pyrb.pitch_shift(
                dados_wav, 24000, 1.5, rbargs={"-F":"-F", "-t":0.65, "-c":6}
    )

    return dados_melhor


if __name__ == "__main__":
    import scipy.io.wavfile
    import playsound
    import time
    import os

    dados = gerarAudio("Oi! Eu sou a Lisa")
    
    scipy.io.wavfile.write("exemplo.wav", 24000, dados)

    playsound.playsound("exemplo.wav")
    os.remove("exemplo.wav")
