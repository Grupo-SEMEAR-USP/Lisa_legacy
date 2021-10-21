import spacy
import string
from spacy.lang.pt.stop_words import STOP_WORDS
stop_words = STOP_WORDS

pontuacoes = string.punctuation

pln_tokenizer = spacy.load("pt_core_news_sm")

def preprocessamento(texto):
	texto = texto.lower()		#Deixa texto somente com minúsculas
	documento = pln_tokenizer(texto)

	lista = []
	for token in documento:	#Lematiza todas as palavras do banco de dados (não é muito preciso)
		#lista.append(token.text)
		lista.append(token.lemma_)

	#Tira stop words e pontuação
	lista = [palavra for palavra in lista if palavra not in stop_words and palavra not in pontuacoes]

	#Junta todos os elementos da lista em 1 string e add epaço entre os elementos
	lista = ' '.join([str(elemento) for elemento in lista if not elemento.isdigit()])
	
	return lista

text = "Estou cansada"     #Digitar o comando para testar o modelo

nlp = spacy.load(".\\textcat_demo\\textcat_demo\\training\\model-last")

doc = nlp(preprocessamento(text))

for k in sorted(doc.cats, key=doc.cats.get, reverse=True):
	print(k, doc.cats[k])
#print(doc.cats)     #Print da probabilidade de cada label

print("\n")
print(preprocessamento(text))
print(text)         #Print do comando digitado acima