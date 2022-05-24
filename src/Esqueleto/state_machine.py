from comandos import comando
import random
import pandas as pd
from os.path import join

def prime(fn):
	def wrapper(*args, **kwargs):
		v = fn(*args, **kwargs)
		v.send(None)
		return v
	return wrapper

class MaqEstados:
	def __init__(self):
		# initializing states
		self.neutro = (self._create_neutro(), "Neutro")
		self.bravo = (self._create_bravo(), "Bravo")
		self.triste = (self._create_triste(), "Triste")
		self.feliz = (self._create_feliz(), "Feliz")
		self.dançando = (self._create_dançando(), "Dançando")
		self.dormindo = (self._create_dormindo(), "Dormindo")
		self.handsup = (self._create_handsup(), "handsup")
		self.falar_nome = (self._create_falar_nome(), "Falar_nome")
		self.falar_projeto = (self._create_falar_projeto(), "Falar_projeto")
		self.soletrando = (self._create_soletrando(), "Soletrando")
		self.falar_ADA = (self._create_falar_ADA(),"Falar_sobre_ADA")
		self.falar_SEMEAR = (self._create_falar_SEMEAR(),"Falar_sobre_SEMEAR")
		self.piada_simples = (self._create_piada_simples(), "Piada sem transição")
		self.piada_pergunta_e_resposta = (self._create_piada_pergunta_e_resposta(), "Piada pergunta e resposta")
		self.piada_toc_toc = (self._create_piada_toc_toc(), "Piada sem toc toc")
		self.piada = (self._create_piada(), "Piada")
		self.tocar_musica = (self._create_tocar_musica(), "Tocar musica")
		
		# setting current state of the system
		self.current_state = self.neutro
		self.previous_state = self.neutro

		# Stopped flag to denote that iteration is Stopped due to bad
		# input against which transition was not defined.
		self.Stopped = False

		# open jokes file
		self.arquivo_pergunta_e_resposta = pd.read_csv(join("res", "piadas", "PIADA_Pergunta_e_Resposta.tsv"), sep="	")
		self.arquivo_sem_transicao = pd.read_csv(join("res", "piadas", "PIADA_Sem_transição.tsv"), sep="	")
		self.arquivo_toc_toc = pd.read_csv(join("res", "piadas", "PIADA_Toc_toc.tsv"), sep="	")

	def send(self, msg):
		"""The function sends the current input to the current state
		It captures the StopIteration exception and marks the Stopped flag.
		"""
		try:
			if(msg.isnumeric()):
				self.current_state[0].send(int(msg))
			else:
				self.current_state[0].send(msg)
		except StopIteration:
			print("Finalizou")
			self.Stopped = True
		
	def does_match(self):
		"""The function at any point in time returns if till the current input
		the string matches the given regular expression.

		It does so by comparing the current state with the end state `q3`.
		It also checks for `Stopped` flag which sees that due to bad input the iteration of FSM had to be Stopped.
		"""
		if self.Stopped:
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
			elif msg == comando.Dancar:
				print('┗(＾0＾)┓')
				print("-> Dançando\n")
				self.current_state = self.dançando
			elif msg == comando.Levantar_bracos:
				print('┗(＾0＾)┗')
				print("-> Levantar Braço\n")
				self.current_state = self.handsup
			elif msg == comando.Falar_nome:
				print("Meu nome é 'Insira um nome de robô aqui'")
				print("-> Falar Nome\n")
				self.current_state = self.falar_nome
			elif msg == comando.Falar_projeto:
				print("Eu sou um robô humanoide ... Mais texto sobre o projeto")
				print("-> Falar Projeto\n")
				self.current_state = self.falar_projeto
			elif msg == comando.Soletrar:
				print("-> Soletrando\nO que você quer que eu soletre?\n")
				self.current_state = self.soletrando
			elif msg == comando.ADA:
				print("Falando sobre a ADA lalalala")
				print("-> Falar sobre ADA\n")
				self.current_state = self.falar_ADA
			elif msg == comando.SEMEAR:
				print("Falando sobre o SEMEAR lalalala")
				print("-> Falar sobre SEMEAR\n")
				self.current_state = self.falar_SEMEAR
			elif msg == comando.Piada:
				self.current_state = self.piada
				self.current_state[0].send(0)
			elif msg == comando.Musica:
				print("-> Escolhendo musica para tocar\n")
				#Parte do código para escolher a musica que irar tocar
				self.current_state = self.tocar_musica
			else:
				# Qualquer outra coisa ele dorme
				print('Vou dormir\n')
				print("-> Dormindo\n")
				self.current_state = self.dormindo

			self.previous_state = self.neutro

	@prime
	def _create_bravo(self):
		while True:
			msg = yield

			if msg == comando.Desculpa:
				print("ok...")
				print("-> Neutro\n")
				self.current_state = self.neutro
			elif msg == comando.Levantar_bracos:
				print('┗(＾0＾)┗')
				print("-> Levantar Braço\n")
				self.current_state = self.handsup
			elif msg == comando.Falar_nome:
				print("Meu nome é 'Insira um nome de robô aqui'")
				print("-> Falar Nome\n")
				self.current_state = self.falar_nome
			elif msg == comando.Falar_projeto:
				print("Eu sou um robô humanoide ... Mais texto sobre o projeto")
				print("-> Falar Projeto\n")
				self.current_state = self.falar_projeto
			elif msg == comando.Soletrar:
				print("-> Soletrando\nO que você quer que eu soletre?\n")
				self.current_state = self.soletrando
			elif msg == comando.ADA:
				print("Falando sobre a ADA lalalala")
				print("-> Falar sobre ADA\n")
				self.current_state = self.falar_ADA
			elif msg == comando.SEMEAR:
				print("Falando sobre o SEMEAR lalalala")
				print("-> Falar sobre SEMEAR\n")
				self.current_state = self.falar_SEMEAR
			elif msg == comando.Piada:
				self.current_state = self.piada
				self.current_state[0].send(0)
			elif msg == comando.Musica:
				print("-> Escolhendo musica para tocar\n")
				#Parte do código para escolher a musica que irar tocar
				self.current_state = self.tocar_musica

			self.previous_state = self.bravo

	@prime
	def _create_triste(self):
		while True:
			msg = yield

			if msg == comando.elogio:
				print("ok...")
				print("-> Neutro\n")
				self.current_state = self.neutro
			elif msg == comando.Levantar_bracos:
				print('┗(＾0＾)┗')
				print("-> Levantar Braço\n")
				self.current_state = self.handsup
			elif msg == comando.Falar_nome:
				print("Meu nome é 'Insira um nome de robô aqui'")
				print("-> Falar Nome\n")
				self.current_state = self.falar_nome
			elif msg == comando.Falar_projeto:
				print("Eu sou um robô humanoide ... Mais texto sobre o projeto")
				print("-> Falar Projeto\n")
				self.current_state = self.falar_projeto
			elif msg == comando.Soletrar:
				print("-> Soletrando\nO que você quer que eu soletre?\n")
				self.current_state = self.soletrando
			elif msg == comando.ADA:
				print("Falando sobre a ADA lalalala")
				print("-> Falar sobre ADA\n")
				self.current_state = self.falar_ADA
			elif msg == comando.SEMEAR:
				print("Falando sobre o SEMEAR lalalala")
				print("-> Falar sobre SEMEAR\n")
				self.current_state = self.falar_SEMEAR
			elif msg == comando.Piada:
				self.current_state = self.piada
				self.current_state[0].send(0)
			elif msg == comando.Musica:
				print("-> Escolhendo musica para tocar\n")
				#Parte do código para escolher a musica que irar tocar
				self.current_state = self.tocar_musica
				
			self.previous_state = self.triste

	@prime
	def _create_feliz(self):
		while True:
			msg = yield

			if msg == comando.Dancar:
				print('┏(･o･)┛♪┗ (･o･) ┓')
				print("-> Dançando\n")
				self.current_state = self.dançando
			if msg == comando.silencio:
				print('._.')
				print("-> Neutro\n")
				self.current_state = self.neutro
			elif msg == comando.Levantar_bracos:
				print('┗(＾0＾)┗')
				print("-> Levantar Braço\n")
				self.current_state = self.handsup
			elif msg == comando.Falar_nome:
				print("Meu nome é 'Insira um nome de robô aqui'")
				print("-> Falar Nome\n")
				self.current_state = self.falar_nome
			elif msg == comando.Falar_projeto:
				print("Eu sou um robô humanoide ... Mais texto sobre o projeto")
				print("-> Falar Projeto\n")
				self.current_state = self.falar_projeto
			elif msg == comando.Soletrar:
				print("-> Soletrando\nO que você quer que eu soletre?\n")
				self.current_state = self.soletrando
			elif msg == comando.ADA:
				print("Falando sobre a ADA lalalala")
				print("-> Falar sobre ADA\n")
				self.current_state = self.falar_ADA
			elif msg == comando.SEMEAR:
				print("Falando sobre o SEMEAR lalalala")
				print("-> Falar sobre SEMEAR\n")
				self.current_state = self.falar_SEMEAR
			elif msg == comando.Piada:
				self.current_state = self.piada
				self.current_state[0].send(0)
			elif msg == comando.Musica:
				print("-> Escolhendo musica para tocar\n")
				#Parte do código para escolher a musica que irar tocar
				self.current_state = self.tocar_musica
				
			self.previous_state = self.feliz
	
	@prime
	def _create_dançando(self):
		while True:
			msg = yield
			if msg == comando.Stop:
				print('Ok!')
				print("-> Feliz\n")
				self.current_state = self.feliz
				
			self.previous_state = self.dançando
	
	@prime
	def _create_handsup(self):
		while True:
			msg = yield

			if msg == comando.Stop:

				print('Ok!')
				if(random.randint(1,10) < 8):
					self.current_state = self.previous_state
					print("->", self.previous_state[1],"\n")
				else:
					self.current_state = self.feliz
					print("Oba")
					print("-> Feliz\n")
					
			self.previous_state = self.handsup

	@prime
	def _create_falar_nome(self):
		while True:
			msg = yield
			if msg == comando.Stop:
				print("Ok!")
				self.current_state = self.previous_state
				print("->", self.previous_state[1],"\n")
				self.previous_state = self.falar_nome
	
	@prime
	def _create_falar_projeto(self):
		while True:
			msg = yield
			if msg == comando.Stop:
				print("Ok!")
				self.current_state = self.previous_state
				print("->", self.previous_state[1],"\n")
				self.previous_state = self.falar_projeto

	@prime
	def _create_dormindo(self):
		while True:
			msg = yield

			if msg == comando.Acordar:
				print("afs afs")
				print("-> Neutro\n")
				self.current_state = self.neutro
			else:
				print("Me deixa dormir")
				print("-> Dormindo\n")
				
			self.previous_state = self.dormindo

	@prime
	def _create_soletrando(self):
		while True:
			msg = yield
			soletrar = msg + ","
			for letter in msg:
				if letter == "a":
					soletrar = soletrar + ("A, ")
				if letter == "b":
					soletrar = soletrar + ("Bê, ")
				if letter == "c":
					soletrar = soletrar + ("Cê, ")
				if letter == "d":
					soletrar = soletrar + ("Dê, ")
				if letter == "e":
					soletrar = soletrar + ("Ê, ")
				if letter == "f":
					soletrar = soletrar + ("É fi, ")
				if letter == "g":
					soletrar = soletrar + ("Gê, ")
				if letter == "h":
					soletrar = soletrar + ("A gá, ")
				if letter == "i":
					soletrar = soletrar + ("I, ")
				if letter == "j":
					soletrar = soletrar + ("Jota, ")
				if letter == "k":
					soletrar = soletrar + ("Ká, ")
				if letter == "l":
					soletrar = soletrar + ("É li, ")
				if letter == "m":
					soletrar = soletrar + ("Ê mi, ")
				if letter == "n":
					soletrar = soletrar + ("Ê ni, ")
				if letter == "o":
					soletrar = soletrar + ("Ô, ")
				if letter == "p":
					soletrar = soletrar + ("Pê, ")
				if letter == "q":
					soletrar = soletrar + ("Quê, ")
				if letter == "r":
					soletrar = soletrar + ("É ri, ")
				if letter == "s":
					soletrar = soletrar + ("É si, ")
				if letter == "t":
					soletrar = soletrar + ("Tê, ")
				if letter == "u":
					soletrar = soletrar + ("Ú, ")
				if letter == "v":
					soletrar = soletrar + ("Vê, ")
				if letter == "w":
					soletrar = soletrar + ("Dábliu, ")
				if letter == "x":
					soletrar = soletrar + ("Xis, ")
				if letter == "y":
					soletrar = soletrar + ("Ípsilom, ")
				if letter == "z":
					soletrar = soletrar + ("Zê, ")
				if letter == "ç":
					soletrar = soletrar + ("Cê cedilha, ")
				if letter == "á":
					soletrar = soletrar + ("A com acento agudo, ")
				if letter == "ã":
					soletrar = soletrar + ("A com til, ")
				if letter == "â":
					soletrar = soletrar + ("A com acento circunflexo, ")
				if letter == "à":
					soletrar = soletrar + ("A com crase, ")
				if letter == "ó":
					soletrar = soletrar + ("Ô com acento agudo, ")
				if letter == "õ":
					soletrar = soletrar + ("Ô com til, ")
				if letter == "ô":
					soletrar = soletrar + ("Ô com acento circunflexo, ")
				if letter == "é":
					soletrar = soletrar + ("Ê com acento agudo, ")
				if letter == "ê":
					soletrar = soletrar + ("Ê com acento circunflexo, ")
				if letter == "ú":
					soletrar = soletrar + ("Ê com acento agudo, ")
				if letter == "ü":
					soletrar = soletrar + ("Ú com trema, ")
				if letter == "í":
					soletrar = soletrar + ("I com acento agudo, ")
				if letter == "0":
					soletrar = soletrar + ("Zero, ")
				if letter == "1":
					soletrar = soletrar + ("Um, ")
				if letter == "2":
					soletrar = soletrar + ("Dois, ")
				if letter == "3":
					soletrar = soletrar + ("Três, ")
				if letter == "4":
					soletrar = soletrar + ("Quatro, ")
				if letter == "5":
					soletrar = soletrar + ("Cinco, ")
				if letter == "6":
					soletrar = soletrar + ("Seis, ")
				if letter == "7":
					soletrar = soletrar + ("Sete, ")
				if letter == "8":
					soletrar = soletrar + ("Oito, ")
				if letter == "9":
					soletrar = soletrar + ("Nove, ")

			print("A palavra se soletra assim: ",soletrar, "\n->",self.previous_state[1])
			self.current_state = self.previous_state
			self.previous_state = self.soletrando
		
	@prime
	def _create_falar_ADA(self):
		while True:
			msg = yield

			if msg == comando.Stop:
				print("Ok!")
				self.current_state = self.previous_state
				print("->", self.previous_state[1],"\n")
			self.previous_state = self.falar_ADA
	
	@prime
	def _create_falar_SEMEAR(self):
		while True:
			msg = yield

			if msg == comando.Stop:
				print("Ok!")
				self.current_state = self.previous_state
				print("->", self.previous_state[1],"\n")
			self.previous_state = self.falar_SEMEAR

	@prime
	def _create_piada_simples(self):
		while True:
			msg = yield
			print(self.arquivo_sem_transicao.shape)
			n = self.arquivo_sem_transicao.shape[0]
			piada = random.randint(0,n-1)
			print(self.arquivo_sem_transicao.iloc[piada][0])
			self.current_state = self.feliz
			
	@prime
	def _create_piada_pergunta_e_resposta(self):
		while True:
			msg = yield
			print(self.arquivo_pergunta_e_resposta.shape)
			n = self.arquivo_pergunta_e_resposta.shape[0]
			piada = random.randint(0,n-1)
			print(self.arquivo_pergunta_e_resposta.iloc[piada][0])

			msg = yield
			if (msg == 2):
				print(self.arquivo_pergunta_e_resposta.iloc[piada][1])
				self.current_state = self.feliz
			elif (msg == 5):
				#tentar fazer em PLN
				self.current_state = self.feliz
			elif (msg == 7):
				self.current_state = self.bravo
			
	@prime
	def _create_piada(self):
		while True:
			msg = yield
			n = random.randint(1,3)
			print(n)

			if(n == 1):
				self.current_state = self.piada_pergunta_e_resposta
			elif(n == 2):
				self.current_state = self.piada_simples
			else:
				self.current_state = self.piada_toc_toc

			self.current_state[0].send(0)
	
	@prime
	def _create_piada_toc_toc(self):
		while True:
			msg = yield

			n = self.arquivo_toc_toc.shape[0]
			piada = random.randint(0,n-1)
			print(self.arquivo_toc_toc.iloc[piada][0])
			msg = yield
			if (msg == 1):
				print(self.arquivo_toc_toc.iloc[piada][2])
				msg = yield
				print(self.arquivo_toc_toc.iloc[piada][4])
				self.current_state = self.feliz
			elif (msg == 2):
				self.current_state = self.bravo

	@prime
	def _create_tocar_musica(self):
		while True:
			msg = yield

			if msg == comando.Stop:
				print('Ok!')
				#Chance dessa parte do código mudar por conta da máquina de estados que pensamos originalmente
				#Rever trecho do código após definir como serão armazenadas as músicas
				if(random.randint(1,10) < 2):	#Chance do robô voltar à emoção antiga
					self.current_state = self.previous_state
					print("->", self.previous_state[1],"\n")
				else:
					self.current_state = self.feliz
					print("Eu amo essa música!!\n")
					print("-> Feliz\n")
					
			self.previous_state = self.tocar_musica
		


			

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
