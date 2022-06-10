import sounddevice as sd
import soundfile as sf
import queue
import os


def gravarAudioArquivo(arq, samplerate=44100, disp=sd.default.device, 
        canais=1):
    '''
    A função gravarAudioArquivo grava um áudio do usuário até ele apertar
    ctrl+C
    '''

    q = queue.Queue()

    def pegarAudio(data, *args):
        q.put(data.copy())

    if os.path.exists(arq):
        os.remove(arq)
    arq = sf.SoundFile(arq, mode='x', samplerate=samplerate,
            channels=canais)

    with sd.InputStream(samplerate=samplerate, 
            device=disp, channels=canais, callback=pegarAudio):
    
        try:
            print("aperte Ctrl+C para parar de gravar")
            while True:
                arq.write(q.get())
        except KeyboardInterrupt:
            arq.close()
            print()


if __name__ == "__main__":
    import playsound
    import os

    gravarAudioArquivo("exemplo.wav")
    playsound.playsound("exemplo.wav")
    os.remove("exemplo.wav")
    