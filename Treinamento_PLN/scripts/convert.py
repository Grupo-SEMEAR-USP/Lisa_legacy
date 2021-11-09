"""Convert textcat annotation from JSONL to spaCy v3 .spacy format."""
from spacy.lang.pt.stop_words import STOP_WORDS		#importar as stop words
from spacy_langdetect  import LanguageDetector
from spacy.language import Language
from spacy.tokens import DocBin
from pathlib import Path

import pandas as pd
import numpy as np
import warnings
import string
import random
import srsly
import spacy
import json
import typer

def preprocessamento(texto):
	global nlp
	texto = texto.lower()		#Deixa texto somente com minúsculas
	documento = nlp(texto)

	lista = []
	for token in documento:	#Lematiza todas as palavras do banco de dados (não é muito preciso)
		#lista.append(token.text)
		lista.append(token.lemma_)

	#Tira stop words e pontuação
	lista = [palavra for palavra in lista if palavra not in STOP_WORDS and palavra not in string.punctuation]

	#Junta todos os elementos da lista em 1 string e add epaço entre os elementos
	lista = ' '.join([str(elemento) for elemento in lista if not elemento.isdigit()])

	return lista

def convert(output_path):
	global nlp
	db = DocBin()
	for line in srsly.read_jsonl("db.json"):
		doc = nlp.make_doc(line["text"])
		doc.cats = line["cats"]
		db.add(doc)
	db.to_disk(output_path)

def csvToJson(input_path):
	#carregamento da base de dados
	baseDeDados = pd.read_csv(input_path, encoding = 'utf-8')
	baseDeDados = baseDeDados.astype(str)

	#Pocessamento na nossa base de dados
	baseDeDados['texto'] = baseDeDados['texto'].apply(preprocessamento)
	baseDeDadosFinal = []

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

		arq.write("{\"text\":\"" + texto + "\",\"cats\":")
		arq.write(json.dumps(dic,ensure_ascii=False))
		arq.write("}\n")
	arq.close()

def main(lang: str, input_path: Path, output_path: Path):
	global nlp
	nlp = spacy.load("pt_core_news_sm")
	nlp.Defaults.stop_words.remove("estar")

	csvToJson(input_path)
	convert(output_path)


if __name__ == "__main__":
	typer.run(main)
