import spacy
import string
from spacy.lang.pt.stop_words import STOP_WORDS
from comandos import comando


def preprocessamento(texto, stop_words, tokenizer):
	pontuacoes = string.punctuation
	texto = texto.lower()		#Deixa texto somente com minúsculas
	documento = tokenizer(texto)

	lista = []
	for token in documento:	#Lematiza todas as palavras do banco de dados (não é muito preciso)
		#lista.append(token.text)
		lista.append(token.lemma_)

	#Tira stop words e pontuação
	#print(lista)
	lista = [palavra for palavra in lista if palavra not in stop_words and palavra not in pontuacoes]

	#Junta todos os elementos da lista em 1 string e add epaço entre os elementos
	lista = ' '.join([str(elemento) for elemento in lista if not elemento.isdigit()])
	
	return lista

def transformCommand(cmd):
	print(cmd)
	if cmd=="DANÇAR":
		return comando.Dancar

	if cmd == "ACORDAR":
		return comando.Acordar
	
	if cmd == "DORMIR":
		return comando.Dormir

	if cmd == "GIRAR":
		return comando.Girar

	if cmd == "LEVANTAR BRAÇOS":
		return comando.Levantar_bracos

	if cmd == "PIADA":
		return comando.Piada

	if cmd == "ESTÁ BEM":
		return comando.Esta_bem

	if cmd == "APRESENTAR":
		return comando.Apresentar

	if cmd == "FALAR SOBRE SEMEAR":
		return comando.SEMEAR

	if cmd == "FALAR SOBRE ADA":
		return comando.ADA

	if cmd == "MÚSICA":
		return comando.Musica

	if cmd == "SOLETRAR":
		return comando.Soletrar

	if cmd == "FALAR O NOME":
		return comando.Falar_nome

def pln(texto, nlp, stop_words, tokenizer):
	doc = nlp(preprocessamento(texto, stop_words, tokenizer))

	for k in sorted(doc.cats, key=doc.cats.get, reverse=True):
		print(k, doc.cats[k])
	#print(doc.cats)     #Print da probabilidade de cada label

	print("\n")
	print(preprocessamento(texto, stop_words, tokenizer))
	print(texto)         #Print do comando digitado acima
	k = sorted(doc.cats, key=doc.cats.get, reverse=True)[0]
	valorComando = transformCommand(k)
	return valorComando