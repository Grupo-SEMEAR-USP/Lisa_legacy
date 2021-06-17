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
			if msg == "xingar":
				if(random.randint(1,10) < 8):
					self.current_state = self.bravo
					print("Inveja mata, sabia? -> Bravo\n")
				else:
					self.current_state = self.triste
					print("ok então ;-; -> Triste\n")
			elif msg == 'elogio':
				print(':D -> Feliz\n')
				self.current_state = self.feliz
			elif msg == 'dança':
				print('┗(＾0＾)┓ -> Dançando\n')
				self.current_state = self.dançando
			else:
				# Qualquer outra coisa ele dorme
				print('Vou dormir -> Dormindo\n')
				self.current_state = self.dormindo
				break

	@prime
	def _create_bravo(self):
		while True:
			msg = yield

			if msg == 'desculpa':
				print("ok... -> Neutro\n")
				self.current_state = self.neutro
			else:
				# on receiving any other input, break the loop
				# so that next time when someone sends any input to
				# the coroutine it raises StopIteration
				break

	@prime
	def _create_triste(self):
		while True:
			msg = yield

			if msg == 'elogio':
				print("ok... -> neutro\n")
				self.current_state = self.neutro
			else:
				# on receiving any other input, break the loop
				# so that next time when someone sends any input to
				# the coroutine it raises StopIteration
				break

	@prime
	def _create_feliz(self):
		while True:
			msg = yield

			if msg == 'dança':
				print('┏(･o･)┛♪┗ (･o･) ┓ -> Dançando\n')
				self.current_state = self.dançando
			if msg == '...':
				print('._. -> Neutro\n')
				self.current_state = self.neutro
	
	@prime
	def _create_dançando(self):
		while True:
			msg = yield
			if msg == 'stop':
				print('Ok! -> Feliz\n')
				self.current_state = self.feliz
	
	@prime
	def _create_dormindo(self):
		while True:
			msg = yield

			print("Me deixa dormir -> Dormindo\n")


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