import sounddevice as sd
import soundfile as sf
import queue
import os


def gravarAudioArquivo(file, samplerate=44100, disp=sd.default.device, 
        canais=1):

    q = queue.Queue()

    def pegarAudio(data, *args):
        q.put(data.copy())

    if os.path.exists(file):
        os.remove(file)
    arq = sf.SoundFile(file, mode='x', samplerate=samplerate,
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
    