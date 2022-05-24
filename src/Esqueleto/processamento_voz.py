import os
import time
import speech_recognition as sr
import pyaudio

import sintese_voz

import threading
import argparse
import tempfile
import queue
import time
import sys
import os

import sounddevice as sd
import soundfile as sf
import numpy  # Make sure NumPy is loaded before it is used in the callback


parser = argparse.ArgumentParser(add_help=False)
parser.add_argument(
	'-l', '--list-devices', action='store_true',
	help='show list of audio devices and exit')
args, remaining = parser.parse_known_args()
if args.list_devices:
	print(sd.query_devices())
	parser.exit(0)



q = queue.Queue()

def callback(indata, frames, time, status):
	"""This is called (from a separate thread) for each audio block."""
	if status:
		print(status, file=sys.stderr)
	q.put(indata.copy())


def threadRecord(filename, device, samplerate, channels, subtype):
	t = threading.currentThread()

	if samplerate is None:
		device_info = sd.query_devices(device, 'input')
		# soundfile expects an int, sounddevice provides a float:
		samplerate = int(device_info['default_samplerate'])

	if filename is None:
		filename = tempfile.mktemp(prefix='delme_rec_unlimited_',
										suffix='.wav', dir='')
	else:
		if(os.path.exists(filename)):
			os.remove(filename)

	# Make sure the file is opened before recording anything:
	with sf.SoundFile(filename, mode='x', samplerate=samplerate, channels=channels, subtype=subtype) as file:
		with sd.InputStream(samplerate=samplerate, device=device, channels=channels, callback=callback):
			while getattr(t, "do_run", True):
				file.write(q.get())
			file.close()

def Record(filename, device, samplerate, channels, subtype):
	thread = threading.Thread(target = threadRecord, args = (filename, device, samplerate, channels, subtype))
	thread.start()

	input("Aperte enter para parar de gravar")

	thread.do_run = False
	time.sleep(1)

def get_audio(filename, device = sd.default.device, samplerate = 44100, channels = 1, subtype = 'PCM_16'):
	Record(filename, device, samplerate, channels, subtype)

	r = sr.Recognizer()
	with sr.AudioFile(filename) as source:
		audio = r.record(source)
		said = ""

		try:
			said = r.recognize_google(audio, language = 'pt-BR')
			print(said)
		except Exception as e:
			print("Exception " + str(e))
	return said

if __name__ == '__main__':
	filename = "audio.wav"
	device = 1
	samplerate = 44100
	channels = 1
	subtype = 'PCM_16'

	while True:
		comando = input()

		if comando == "gravar":
			text = get_audio(filename = filename).lower()
			sintese_voz.speak(text)
			print(text)
