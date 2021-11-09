import pandas as pd
import srsly
import string
import spacy
import random
import seaborn as sns
import numpy as np
import matplotlib.pyplot as plt
from spacy.language import Language
from spacy_langdetect  import LanguageDetector
import json
import typer
import warnings
from pathlib import Path
from spacy.tokens import DocBin
from spacy.lang.pt.stop_words import STOP_WORDS		#importar as stop words

nlp = spacy.load("pt_core_news_sm")
print(nlp.pipe_names)


#carregamento da base de dados
baseDeDados = pd.read_csv("DataBaseFinal.csv", encoding = 'utf-8')
baseDeDados = baseDeDados.astype(str)

#preparando o processamento de texto
#importar a pontuação
pontuacoes = string.punctuation
#check das pontuações
#print(pontuacoes)


nlp.Defaults.stop_words.remove("estar")
stop_words = STOP_WORDS

""" função para fazer o pré-processamento de texto
	remoção de caracteres especiais
	tokenização
	lematização
	remoção de stop words """
def preprocessamento(texto):
	texto = texto.lower()		#Deixa texto somente com minúsculas
	documento = nlp(texto)

	lista = []
	for token in documento:	#Lematiza todas as palavras do banco de dados (não é muito preciso)
		#lista.append(token.text)
		lista.append(token.lemma_)

	#Tira stop words e pontuação
	lista = [palavra for palavra in lista if palavra not in stop_words and palavra not in pontuacoes]

	#Junta todos os elementos da lista em 1 string e add epaço entre os elementos
	lista = ' '.join([str(elemento) for elemento in lista if not elemento.isdigit()])

	return lista

def convert(json_path, output):
	db = DocBin()
	for line in srsly.read_jsonl(json_path):
		doc = nlp.make_doc(line["text"])
		doc.cats = line["cats"]
		db.add(doc)
	db.to_disk(output)

#Pocessamento na nossa base de dados
baseDeDados['texto'] = baseDeDados['texto'].apply(preprocessamento)
baseDeDadosFinal = []
db = DocBin()

arq = open("db.json", "w", encoding='utf8')

#criação do dicionário
for texto, Comando in zip(baseDeDados['texto'],baseDeDados['Comando']):

	if Comando == 'Acordar':
		dic = dict({'ACORDAR': 1.0, 'DORMIR': 0.0, 'GIRAR': 0.0, 'LEVANTAR BRAÇOS': 0.0, 'PIADA': 0.0, 
		'FALAR O NOME': 0.0, 'ESTÁ BEM': 0.0, 'APRESENTAR': 0.0,'FALAR SOBRE SEMEAR': 0.0, 
		'FALAR SOBRE ADA': 0.0, 'MÚSICA': 0.0, 'SOLETRAR': 0.0, 'DANÇAR': 0.0 })
	elif Comando == 'Dormir':
		dic = dict({'ACORDAR': 0.0, 'DORMIR': 1.0, 'GIRAR': 0.0, 'LEVANTAR BRAÇOS': 0.0, 'PIADA': 0.0, 
		'FALAR O NOME': 0.0, 'ESTÁ BEM': 0.0, 'APRESENTAR': 0.0,'FALAR SOBRE SEMEAR': 0.0, 
		'FALAR SOBRE ADA': 0.0, 'MÚSICA': 0.0, 'SOLETRAR': 0.0, 'DANÇAR': 0.0 })
	elif Comando == 'Girar':
		dic = dict({'ACORDAR': 0.0, 'DORMIR': 0.0, 'GIRAR': 1.0, 'LEVANTAR BRAÇOS': 0.0, 'PIADA': 0.0, 
		'FALAR O NOME': 0.0, 'ESTÁ BEM': 0.0, 'APRESENTAR': 0.0,'FALAR SOBRE SEMEAR': 0.0, 
		'FALAR SOBRE ADA': 0.0, 'MÚSICA': 0.0, 'SOLETRAR': 0.0, 'DANÇAR': 0.0 })
	elif Comando == 'Levantar braços':
		dic = dict({'ACORDAR': 0.0, 'DORMIR': 0.0, 'GIRAR': 0.0, 'LEVANTAR BRAÇOS': 1.0, 'PIADA': 0.0, 
		'FALAR O NOME': 0.0, 'ESTÁ BEM': 0.0, 'APRESENTAR': 0.0,'FALAR SOBRE SEMEAR': 0.0, 
		'FALAR SOBRE ADA': 0.0, 'MÚSICA': 0.0, 'SOLETRAR': 0.0, 'DANÇAR': 0.0 })
	elif Comando == 'Piada':
		dic = dict({'ACORDAR': 0.0, 'DORMIR': 0.0, 'GIRAR': 0.0, 'LEVANTAR BRAÇOS': 0.0, 'PIADA': 1.0, 
		'FALAR O NOME': 0.0, 'ESTÁ BEM': 0.0, 'APRESENTAR': 0.0,'FALAR SOBRE SEMEAR': 0.0, 
		'FALAR SOBRE ADA': 0.0, 'MÚSICA': 0.0, 'SOLETRAR': 0.0, 'DANÇAR': 0.0 })
	elif Comando == 'Falar o nome':
		dic = dict({'ACORDAR': 0.0, 'DORMIR': 0.0, 'GIRAR': 0.0, 'LEVANTAR BRAÇOS': 0.0, 'PIADA': 0.0, 
		'FALAR O NOME': 1.0, 'ESTÁ BEM': 0.0, 'APRESENTAR': 0.0,'FALAR SOBRE SEMEAR': 0.0, 
		'FALAR SOBRE ADA': 0.0, 'MÚSICA': 0.0, 'SOLETRAR': 0.0, 'DANÇAR': 0.0 })
	elif Comando == 'Está bem':
		dic = dict({'ACORDAR': 0.0, 'DORMIR': 0.0, 'GIRAR': 0.0, 'LEVANTAR BRAÇOS': 0.0, 'PIADA': 0.0, 
		'FALAR O NOME': 0.0, 'ESTÁ BEM': 1.0, 'APRESENTAR': 0.0,'FALAR SOBRE SEMEAR': 0.0, 
		'FALAR SOBRE ADA': 0.0, 'MÚSICA': 0.0, 'SOLETRAR': 0.0, 'DANÇAR': 0.0 })
	elif Comando == 'Apresentar':
		dic = dict({'ACORDAR': 0.0, 'DORMIR': 0.0, 'GIRAR': 0.0, 'LEVANTAR BRAÇOS': 0.0, 'PIADA': 0.0, 
		'FALAR O NOME': 0.0, 'ESTÁ BEM': 0.0, 'APRESENTAR': 1.0,'FALAR SOBRE SEMEAR': 0.0, 
		'FALAR SOBRE ADA': 0.0, 'MÚSICA': 0.0, 'SOLETRAR': 0.0, 'DANÇAR': 0.0 })
	elif Comando == 'Falar sobre SEMEAR':
		dic = dict({'ACORDAR': 0.0, 'DORMIR': 0.0, 'GIRAR': 0.0, 'LEVANTAR BRAÇOS': 0.0, 'PIADA': 0.0, 
		'FALAR O NOME': 0.0, 'ESTÁ BEM': 0.0, 'APRESENTAR': 0.0,'FALAR SOBRE SEMEAR': 1.0, 
		'FALAR SOBRE ADA': 0.0, 'MÚSICA': 0.0, 'SOLETRAR': 0.0, 'DANÇAR': 0.0 })
	elif Comando == 'Falar sobre ADA':
		dic = dict({'ACORDAR': 0.0, 'DORMIR': 0.0, 'GIRAR': 0.0, 'LEVANTAR BRAÇOS': 0.0, 'PIADA': 0.0, 
		'FALAR O NOME': 0.0, 'ESTÁ BEM': 0.0, 'APRESENTAR': 0.0,'FALAR SOBRE SEMEAR': 0.0, 
		'FALAR SOBRE ADA': 1.0, 'MÚSICA': 0.0, 'SOLETRAR': 0.0, 'DANÇAR': 0.0 })
	elif Comando == 'Música':
		dic = dict({'ACORDAR': 0.0, 'DORMIR': 0.0, 'GIRAR': 0.0, 'LEVANTAR BRAÇOS': 0.0, 'PIADA': 0.0, 
		'FALAR O NOME': 0.0, 'ESTÁ BEM': 0.0, 'APRESENTAR': 0.0,'FALAR SOBRE SEMEAR': 0.0, 
		'FALAR SOBRE ADA': 0.0, 'MÚSICA': 1.0, 'SOLETRAR': 0.0, 'DANÇAR': 0.0 })
	elif Comando == 'Soletrar':
		dic = dict({'ACORDAR': 0.0, 'DORMIR': 0.0, 'GIRAR': 0.0, 'LEVANTAR BRAÇOS': 0.0, 'PIADA': 0.0, 
		'FALAR O NOME': 0.0, 'ESTÁ BEM': 0.0, 'APRESENTAR': 0.0,'FALAR SOBRE SEMEAR': 0.0, 
		'FALAR SOBRE ADA': 0.0, 'MÚSICA': 0.0, 'SOLETRAR': 1.0, 'DANÇAR': 0.0 })
	elif Comando == 'Dançar':
		dic = dict({'ACORDAR': 0.0, 'DORMIR': 0.0, 'GIRAR': 0.0, 'LEVANTAR BRAÇOS': 0.0, 'PIADA': 0.0, 
		'FALAR O NOME': 0.0, 'ESTÁ BEM': 0.0, 'APRESENTAR': 0.0,'FALAR SOBRE SEMEAR': 0.0, 
		'FALAR SOBRE ADA': 0.0, 'MÚSICA': 0.0, 'SOLETRAR': 0.0, 'DANÇAR': 1.0 })

	doc = nlp.make_doc(texto)
	doc.cats = dic.copy()
	#db.add(doc)

	arq.write("{\"text\":\"" + texto + "\",\"cats\":")
	arq.write(json.dumps(dic,ensure_ascii=False))
	arq.write("}\n")

	#print(doc.cats)

	#baseDeDadosFinal.append([texto, dic.copy()])


arq.close()

convert("db.json", "db.spacy")
db.to_disk("db")

#Após rodar o PLN.py, rodar a linha de comando abaixo
#python -m spacy train config.conf --output training/ --paths.train db.spacy --paths.dev db.spacy --nlp.lang "pt" --gpu-id -1
#python -m spacy debug data config.conf --paths.train db --paths.dev db --nlp.lang "pt"
#python -m spacy init fill-config config.conf

#Site de referência:
#https://towardsdatascience.com/sarcasm-text-classification-using-spacy-in-python-7cd39074f32e