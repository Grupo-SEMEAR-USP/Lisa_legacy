from comandos import comando
import random

def prime(fn):
    def wrapper(*args, **kwargs):
        v = fn(*args, **kwargs)
        v.send(None)
        return v
    return wrapper

class MaqEstados:
	def __init__(self):
		# initializing states
		self.neutro = self._create_neutro()
		self.bravo = self._create_bravo()
		self.triste = self._create_triste()
		self.feliz = self._create_feliz()
		self.dançando = self._create_dançando()
		self.dormindo = self._create_dormindo()
		
		# setting current state of the system
		self.current_state = self.neutro

		# stopped flag to denote that iteration is stopped due to bad
		# input against which transition was not defined.
		self.stopped = False

	def send(self, msg):
		"""The function sends the current input to the current state
		It captures the StopIteration exception and marks the stopped flag.
		"""
		try:
			self.current_state.send(msg)
		except StopIteration:
			print("Finalizou")
			self.stopped = True
		
	def does_match(self):
		"""The function at any point in time returns if till the current input
		the string matches the given regular expression.

		It does so by comparing the current state with the end state `q3`.
		It also checks for `stopped` flag which sees that due to bad input the iteration of FSM had to be stopped.
		"""
		if self.stopped:
			return False
		return self.current_state == self.q3
	
	@prime
	def _create_neutro(self):
		while True:
			# Wait till the input is received.
			# once received store the input in `msg`
			msg = yield

			# depending on what we received as the input
			# change the current state of the fsm
			if msg == comando.ofensa:
				if(random.randint(1,10) < 8):
					self.current_state = self.bravo
					print("Inveja mata, sabia?")
					print("-> Bravo\n")
				else:
					self.current_state = self.triste
					print("ok então ;-;")
					print("-> Triste\n")
			elif msg == comando.elogio:
				print(':D')
				print("-> Feliz\n")
				self.current_state = self.feliz
			elif msg == comando.dance:
				print('┗(＾0＾)┓')
				print("-> Dançando\n")
				self.current_state = self.dançando
			else:
				# Qualquer outra coisa ele dorme
				print('Vou dormir\n')
				print("-> Dormindo\n")
				self.current_state = self.dormindo

	@prime
	def _create_bravo(self):
		while True:
			msg = yield

			if msg == comando.desculpa:
				print("ok...")
				print("-> Neutro\n")
				self.current_state = self.neutro

	@prime
	def _create_triste(self):
		while True:
			msg = yield

			if msg == comando.elogio:
				print("ok...")
				print("-> Neutro\n")
				self.current_state = self.neutro

	@prime
	def _create_feliz(self):
		while True:
			msg = yield

			if msg == comando.dance:
				print('┏(･o･)┛♪┗ (･o･) ┓')
				print("-> Dançando\n")
				self.current_state = self.dançando
			if msg == comando.silencio:
				print('._.')
				print("-> Neutro\n")
				self.current_state = self.neutro
	
	@prime
	def _create_dançando(self):
		while True:
			msg = yield
			if msg == comando.stop:
				print('Ok!')
				print("-> Feliz\n")
				self.current_state = self.feliz
	
	@prime
	def _create_dormindo(self):
		while True:
			msg = yield

			if msg == comando.acordar:
				print("afs afs")
				print("-> Neutro\n")
				self.current_state = self.neutro
			else:
				print("Me deixa dormir")
				print("-> Dormindo\n")


def main():
	robo = MaqEstados()
	print("criado")

	if(comando.elogio == 1):
		print("oe")

	while True:
		msg = input('Mensagem: ')

		robo.send(msg)

if (__name__ == '__main__'):
	main()