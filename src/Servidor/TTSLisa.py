import gtts
import pydub
import pyrubberband as pyrb
import numpy as np
import io
import structlog

logger = structlog.get_logger("TTS")

def gerarAudio(texto, formato_saida=np.uint8):
    '''
    A função gerarAudio cria um áudio de Text to Speech a partir de um texto,
    sendo baseado no text to speech do google, mas com uma mudança de voz para
    ser mais amigável e menos cansativo de ouvir. O áudio de retorno é um array
    de numpy contendo todo o áudio descomprimido no formato especificado, que 
    pode ser np.float64, np.int32, np.int16 ou np.int8, com os tipos inteiros 
    tendo variações unsigned.
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
    
    if formato_saida == np.float64:
        return dados_melhor

    # convertendo para o tipo especificado (pyrb somente retorna float64)
    if formato_saida == np.int32 or formato_saida == np.uint32:
        multiplicador = 2**31-1
    elif formato_saida == np.int16 or formato_saida == np.uint16:
        multiplicador = 2**15-1
    elif formato_saida == np.int8 or formato_saida == np.uint8:
        multiplicador = 2**7-1
    else:
        logger.warning("tipo não suportado no TTS, default para np.uint8",
            tipo=formato_saida)
        formato_saida = np.uint8
        multiplicador = 2**7-1
    
    
    dados_melhor = dados_melhor*multiplicador
    if formato_saida in [np.uint32, np.uint16, np.uint8]:
        dados_melhor += multiplicador

    return dados_melhor.astype(formato_saida)


if __name__ == "__main__":
    import scipy.io.wavfile
    import playsound
    import time
    import os

    dados = gerarAudio("Oi! Eu sou a Lisa")
    
    scipy.io.wavfile.write("exemplo.wav", 24000, dados)

    playsound.playsound("exemplo.wav")
    os.remove("exemplo.wav")
