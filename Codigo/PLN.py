import pandas as pd
import string
import spacy
import random
import seaborn as sns
import numpy as np
import matplotlib.pyplot as plt
from spacy.language import Language
from spacy_langdetect  import LanguageDetector

import srsly
import typer
import warnings
from pathlib import Path
from spacy.tokens import DocBin

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

#importar as stop words
from spacy.lang.pt.stop_words import STOP_WORDS
stop_words = STOP_WORDS

""" função para fazer o pré-processamento de texto
	remoção de caracteres especiais
	tokenização
	lematização
	remoção de stop words """
def preprocessamento(texto):
	texto = texto.lower()
	documento = nlp(texto)

	lista = []
	for token in documento:
		#lista.append(token.text)
		lista.append(token.lemma_)

	lista = [palavra for palavra in lista if palavra not in stop_words and palavra not in pontuacoes]
	lista = ' '.join([str(elemento) for elemento in lista if not elemento.isdigit()])

	return lista

#aplicando o pre processamento na nossa base de dados
baseDeDados['texto'] = baseDeDados['texto'].apply(preprocessamento)
baseDeDadosFinal = []
db = DocBin()

#criação do dicionário
for texto, Comando in zip(baseDeDados['texto'],baseDeDados['Comando']):

	if Comando == 'Acordar':
		dic = ({'ACORDAR': 1.0, 'DORMIR': 0.0, 'GIRAR': 0.0, 'LEVANTAR BRAÇOS': 0.0, 'PIADA': 0.0, 
		'FALAR O NOME': 0.0, 'ESTÁ BEM': 0.0, 'APRESENTAR': 0.0,'FALAR SOBRE SEMEAR': 0.0, 
		'FALAR SOBRE ADA': 0.0, 'MÚSICA': 0.0, 'SOLETRAR': 0.0, 'DANÇAR': 0.0 })
	elif Comando == 'Dormir':
		dic = ({'ACORDAR': 0.0, 'DORMIR': 1.0, 'GIRAR': 0.0, 'LEVANTAR BRAÇOS': 0.0, 'PIADA': 0.0, 
		'FALAR O NOME': 0.0, 'ESTÁ BEM': 0.0, 'APRESENTAR': 0.0,'FALAR SOBRE SEMEAR': 0.0, 
		'FALAR SOBRE ADA': 0.0, 'MÚSICA': 0.0, 'SOLETRAR': 0.0, 'DANÇAR': 0.0 })
	elif Comando == 'Girar':
		dic = ({'ACORDAR': 0.0, 'DORMIR': 0.0, 'GIRAR': 1.0, 'LEVANTAR BRAÇOS': 0.0, 'PIADA': 0.0, 
		'FALAR O NOME': 0.0, 'ESTÁ BEM': 0.0, 'APRESENTAR': 0.0,'FALAR SOBRE SEMEAR': 0.0, 
		'FALAR SOBRE ADA': 0.0, 'MÚSICA': 0.0, 'SOLETRAR': 0.0, 'DANÇAR': 0.0 })
	elif Comando == 'Levantar braços':
		dic = ({'ACORDAR': 0.0, 'DORMIR': 0.0, 'GIRAR': 0.0, 'LEVANTAR BRAÇOS': 1.0, 'PIADA': 0.0, 
		'FALAR O NOME': 0.0, 'ESTÁ BEM': 0.0, 'APRESENTAR': 0.0,'FALAR SOBRE SEMEAR': 0.0, 
		'FALAR SOBRE ADA': 0.0, 'MÚSICA': 0.0, 'SOLETRAR': 0.0, 'DANÇAR': 0.0 })
	elif Comando == 'Piada':
		dic = ({'ACORDAR': 0.0, 'DORMIR': 0.0, 'GIRAR': 0.0, 'LEVANTAR BRAÇOS': 0.0, 'PIADA': 1.0, 
		'FALAR O NOME': 0.0, 'ESTÁ BEM': 0.0, 'APRESENTAR': 0.0,'FALAR SOBRE SEMEAR': 0.0, 
		'FALAR SOBRE ADA': 0.0, 'MÚSICA': 0.0, 'SOLETRAR': 0.0, 'DANÇAR': 0.0 })
	elif Comando == 'Falar o nome':
		dic = ({'ACORDAR': 0.0, 'DORMIR': 0.0, 'GIRAR': 0.0, 'LEVANTAR BRAÇOS': 0.0, 'PIADA': 0.0, 
		'FALAR O NOME': 1.0, 'ESTÁ BEM': 0.0, 'APRESENTAR': 0.0,'FALAR SOBRE SEMEAR': 0.0, 
		'FALAR SOBRE ADA': 0.0, 'MÚSICA': 0.0, 'SOLETRAR': 0.0, 'DANÇAR': 0.0 })
	elif Comando == 'Está bem':
		dic = ({'ACORDAR': 0.0, 'DORMIR': 0.0, 'GIRAR': 0.0, 'LEVANTAR BRAÇOS': 0.0, 'PIADA': 0.0, 
		'FALAR O NOME': 0.0, 'ESTÁ BEM': 1.0, 'APRESENTAR': 0.0,'FALAR SOBRE SEMEAR': 0.0, 
		'FALAR SOBRE ADA': 0.0, 'MÚSICA': 0.0, 'SOLETRAR': 0.0, 'DANÇAR': 0.0 })
	elif Comando == 'Apresentar':
		dic = ({'ACORDAR': 0.0, 'DORMIR': 0.0, 'GIRAR': 0.0, 'LEVANTAR BRAÇOS': 0.0, 'PIADA': 0.0, 
		'FALAR O NOME': 0.0, 'ESTÁ BEM': 0.0, 'APRESENTAR': 1.0,'FALAR SOBRE SEMEAR': 0.0, 
		'FALAR SOBRE ADA': 0.0, 'MÚSICA': 0.0, 'SOLETRAR': 0.0, 'DANÇAR': 0.0 })
	elif Comando == 'Falar sobre SEMEAR':
		dic = ({'ACORDAR': 0.0, 'DORMIR': 0.0, 'GIRAR': 0.0, 'LEVANTAR BRAÇOS': 0.0, 'PIADA': 0.0, 
		'FALAR O NOME': 0.0, 'ESTÁ BEM': 0.0, 'APRESENTAR': 0.0,'FALAR SOBRE SEMEAR': 1.0, 
		'FALAR SOBRE ADA': 0.0, 'MÚSICA': 0.0, 'SOLETRAR': 0.0, 'DANÇAR': 0.0 })
	elif Comando == 'Falar sobre ADA':
		dic = ({'ACORDAR': 0.0, 'DORMIR': 0.0, 'GIRAR': 0.0, 'LEVANTAR BRAÇOS': 0.0, 'PIADA': 0.0, 
		'FALAR O NOME': 0.0, 'ESTÁ BEM': 0.0, 'APRESENTAR': 0.0,'FALAR SOBRE SEMEAR': 0.0, 
		'FALAR SOBRE ADA': 1.0, 'MÚSICA': 0.0, 'SOLETRAR': 0.0, 'DANÇAR': 0.0 })
	elif Comando == 'Música':
		dic = ({'ACORDAR': 0.0, 'DORMIR': 0.0, 'GIRAR': 0.0, 'LEVANTAR BRAÇOS': 0.0, 'PIADA': 0.0, 
		'FALAR O NOME': 0.0, 'ESTÁ BEM': 0.0, 'APRESENTAR': 0.0,'FALAR SOBRE SEMEAR': 0.0, 
		'FALAR SOBRE ADA': 0.0, 'MÚSICA': 1.0, 'SOLETRAR': 0.0, 'DANÇAR': 0.0 })
	elif Comando == 'Soletrar':
		dic = ({'ACORDAR': 0.0, 'DORMIR': 0.0, 'GIRAR': 0.0, 'LEVANTAR BRAÇOS': 0.0, 'PIADA': 0.0, 
		'FALAR O NOME': 0.0, 'ESTÁ BEM': 0.0, 'APRESENTAR': 0.0,'FALAR SOBRE SEMEAR': 0.0, 
		'FALAR SOBRE ADA': 0.0, 'MÚSICA': 0.0, 'SOLETRAR': 1.0, 'DANÇAR': 0.0 })
	elif Comando == 'Dançar':
		dic = ({'ACORDAR': 0.0, 'DORMIR': 0.0, 'GIRAR': 0.0, 'LEVANTAR BRAÇOS': 0.0, 'PIADA': 0.0, 
		'FALAR O NOME': 0.0, 'ESTÁ BEM': 0.0, 'APRESENTAR': 0.0,'FALAR SOBRE SEMEAR': 0.0, 
		'FALAR SOBRE ADA': 0.0, 'MÚSICA': 0.0, 'SOLETRAR': 0.0, 'DANÇAR': 1.0 })

	doc = nlp.make_doc(texto)
	doc.cats = dic.copy()
	db.add(doc)

	#baseDeDadosFinal.append([texto, dic.copy()])

db.to_disk("db")