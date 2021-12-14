import speech_recognition as sr
import mmap
import time
import os

from playsound import playsound
from gtts import gTTS

from pydub import AudioSegment
from pydub.playback import play


def speak(text):
	print(os.getcwd())
	tts = gTTS(text=text, lang="pt")
	filename = "./voice.mp3"
	tts.save(filename)
	
	path = os.getcwd()+"\\"+filename
	# path.replace('\\','\\\\')


	with open(path) as f:
		song = AudioSegment.from_file(f, format="mp3")
		play(song)

	# mp3 = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ) 

	# playsound(mp3)
	# mixer.init()
	# mixer.music.load(mp3)
	# mixer.music.play()
	# while mixer.music.get_busy():  # wait for music to finish playing
	#     time.sleep(0.1)

	

if __name__ == '__main__':
	text = 'Arrasou miga'
	speak(text)