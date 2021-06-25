import pandas as pd
import string
import spacy
import random
import seaborn as sns
import numpy as np
import matplotlib.pyplot as plt
from spacy.language import Language
from spacy_langdetect  import LanguageDetector

"""
def get_lang_detector(nlp, name):
  return LanguageDetector()
"""

nlp = spacy.load("pt_core_news_sm")
print(nlp.pipe_names)
#check do carregamento
#print(pln)

#carregamento da base de dados
#sheet_ID = '1lKD0M7h1Knaer2nETaVdM1RF80PMx-3Rsorb86NNVO0'
#baseDeDados = pd.read_csv(f"https://docs.google.com/spreadsheets/d/{sheet_ID}/export?format=csv")
baseDeDados = pd.read_csv("DataBaseFinal.csv", encoding = 'utf-8')
baseDeDados = baseDeDados.astype(str)

"""
#checks da base de dados
print(baseDeDados.shape) #verificar quantidade de registros e nro de colunas
print(baseDeDados.head()) #verificar primeiros registros
print(baseDeDados.tail() )#verificar ultimos registros
sns.countplot(y = baseDeDados['Comando'])
plt.show()
"""

#preparando o processamento de texto
    #importar a pontuação
pontuacoes = string.punctuation
    #check das pontuações
#print(pontuacoes)

    #importar as stop words
from spacy.lang.pt.stop_words import STOP_WORDS
stop_words = STOP_WORDS
        # checando se foram importadas corretamente
#print(stop_words)
#print(len(stop_words))

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

""" Testando o pre processamento de texto
teste = preprocessamento('estou fazendo um teste do programa, espero que funcione')
print(teste)
"""

#aplicando o pre processamento na nossa base de dados
baseDeDados['texto'] = baseDeDados['texto'].apply(preprocessamento)

"""
#check do tratamento da base de dados
print(baseDeDados.head(10))
print(baseDeDados.tail())
"""

baseDeDadosFinal = []

#criação do dicionário
for texto, Comando in zip(baseDeDados['texto'],baseDeDados['Comando']):
  #print(texto, Comando)

  if Comando == 'Acordar':
    dic = ({'ACORDAR': True, 'DORMIR': False, 'GIRAR': False, 'LEVANTAR BRAÇOS': False, 'PIADA': False, 
    'FALAR O NOME': False, 'ESTÁ BEM': False, 'APRESENTAR': False,'FALAR SOBRE SEMEAR': False, 
    'FALAR SOBRE ADA': False, 'MÚSICA': False, 'SOLETRAR': False, 'DANÇAR': False })
  elif Comando == 'Dormir':
    dic = ({'ACORDAR': False, 'DORMIR': True, 'GIRAR': False, 'LEVANTAR BRAÇOS': False, 'PIADA': False, 
    'FALAR O NOME': False, 'ESTÁ BEM': False, 'APRESENTAR': False,'FALAR SOBRE SEMEAR': False, 
    'FALAR SOBRE ADA': False, 'MÚSICA': False, 'SOLETRAR': False, 'DANÇAR': False })
  elif Comando == 'Girar':
    dic = ({'ACORDAR': False, 'DORMIR': False, 'GIRAR': True, 'LEVANTAR BRAÇOS': False, 'PIADA': False, 
    'FALAR O NOME': False, 'ESTÁ BEM': False, 'APRESENTAR': False,'FALAR SOBRE SEMEAR': False, 
    'FALAR SOBRE ADA': False, 'MÚSICA': False, 'SOLETRAR': False, 'DANÇAR': False })
  elif Comando == 'Levantar braços':
    dic = ({'ACORDAR': False, 'DORMIR': False, 'GIRAR': False, 'LEVANTAR BRAÇOS': True, 'PIADA': False, 
    'FALAR O NOME': False, 'ESTÁ BEM': False, 'APRESENTAR': False,'FALAR SOBRE SEMEAR': False, 
    'FALAR SOBRE ADA': False, 'MÚSICA': False, 'SOLETRAR': False, 'DANÇAR': False })
  elif Comando == 'Piada':
    dic = ({'ACORDAR': False, 'DORMIR': False, 'GIRAR': False, 'LEVANTAR BRAÇOS': False, 'PIADA': True, 
    'FALAR O NOME': False, 'ESTÁ BEM': False, 'APRESENTAR': False,'FALAR SOBRE SEMEAR': False, 
    'FALAR SOBRE ADA': False, 'MÚSICA': False, 'SOLETRAR': False, 'DANÇAR': False })
  elif Comando == 'Falar o nome':
    dic = ({'ACORDAR': False, 'DORMIR': False, 'GIRAR': False, 'LEVANTAR BRAÇOS': False, 'PIADA': False, 
    'FALAR O NOME': True, 'ESTÁ BEM': False, 'APRESENTAR': False,'FALAR SOBRE SEMEAR': False, 
    'FALAR SOBRE ADA': False, 'MÚSICA': False, 'SOLETRAR': False, 'DANÇAR': False })
  elif Comando == 'Está bem':
    dic = ({'ACORDAR': False, 'DORMIR': False, 'GIRAR': False, 'LEVANTAR BRAÇOS': False, 'PIADA': False, 
    'FALAR O NOME': False, 'ESTÁ BEM': True, 'APRESENTAR': False,'FALAR SOBRE SEMEAR': False, 
    'FALAR SOBRE ADA': False, 'MÚSICA': False, 'SOLETRAR': False, 'DANÇAR': False })
  elif Comando == 'Apresentar':
    dic = ({'ACORDAR': False, 'DORMIR': False, 'GIRAR': False, 'LEVANTAR BRAÇOS': False, 'PIADA': False, 
    'FALAR O NOME': False, 'ESTÁ BEM': False, 'APRESENTAR': True,'FALAR SOBRE SEMEAR': False, 
    'FALAR SOBRE ADA': False, 'MÚSICA': False, 'SOLETRAR': False, 'DANÇAR': False })
  elif Comando == 'Falar sobre SEMEAR':
    dic = ({'ACORDAR': False, 'DORMIR': False, 'GIRAR': False, 'LEVANTAR BRAÇOS': False, 'PIADA': False, 
    'FALAR O NOME': False, 'ESTÁ BEM': False, 'APRESENTAR': False,'FALAR SOBRE SEMEAR': True, 
    'FALAR SOBRE ADA': False, 'MÚSICA': False, 'SOLETRAR': False, 'DANÇAR': False })
  elif Comando == 'Falar sobre ADA':
    dic = ({'ACORDAR': False, 'DORMIR': False, 'GIRAR': False, 'LEVANTAR BRAÇOS': False, 'PIADA': False, 
    'FALAR O NOME': False, 'ESTÁ BEM': False, 'APRESENTAR': False,'FALAR SOBRE SEMEAR': False, 
    'FALAR SOBRE ADA': True, 'MÚSICA': False, 'SOLETRAR': False, 'DANÇAR': False })
  elif Comando == 'Música':
    dic = ({'ACORDAR': False, 'DORMIR': False, 'GIRAR': False, 'LEVANTAR BRAÇOS': False, 'PIADA': False, 
    'FALAR O NOME': False, 'ESTÁ BEM': False, 'APRESENTAR': False,'FALAR SOBRE SEMEAR': False, 
    'FALAR SOBRE ADA': False, 'MÚSICA': True, 'SOLETRAR': False, 'DANÇAR': False })
  elif Comando == 'Soletrar':
    dic = ({'ACORDAR': False, 'DORMIR': False, 'GIRAR': False, 'LEVANTAR BRAÇOS': False, 'PIADA': False, 
    'FALAR O NOME': False, 'ESTÁ BEM': False, 'APRESENTAR': False,'FALAR SOBRE SEMEAR': False, 
    'FALAR SOBRE ADA': False, 'MÚSICA': False, 'SOLETRAR': True, 'DANÇAR': False })
  elif Comando == 'Dançar':
    dic = ({'ACORDAR': False, 'DORMIR': False, 'GIRAR': False, 'LEVANTAR BRAÇOS': False, 'PIADA': False, 
    'FALAR O NOME': False, 'ESTÁ BEM': False, 'APRESENTAR': False,'FALAR SOBRE SEMEAR': False, 
    'FALAR SOBRE ADA': False, 'MÚSICA': False, 'SOLETRAR': False, 'DANÇAR': True })

  baseDeDadosFinal.append([texto, dic.copy()])

"""
#Check do dicionário
print(len(baseDeDadosFinal))
print(baseDeDadosFinal[233][0])
print(baseDeDadosFinal[233][1])
"""



"""
Language.factory("language_detector", func=get_lang_detector)
nlp.add_pipe('language_detector')
"""

#Criando o classificador
modelo = spacy.blank('pt')
print(modelo.pipe_names)

"""
categorias = modelo.create_pipe()
categorias.add_label("ACORDAR")
categorias.add_label("DORMIR")
categorias.add_label("GIRAR")
categorias.add_label("LEVANTAR BRAÇOS")
categorias.add_label("PIADA")
categorias.add_label("FALAR O NOME")
categorias.add_label("ESTÁ BEM")
categorias.add_label("APRESENTAR")
categorias.add_label("FALAR SOBRE SEMEAR")
categorias.add_label("FALAR SOBRE ADA")
categorias.add_label("MÚSICA")
categorias.add_label("SOLETRAR")
categorias.add_label("DANÇAR")
"""
#print(nlp.pipe_names)

"""
ner = nlp.get_pipe("ner")
#ner = modelo.create_pipe('ner')
ner.add_label("ACORDAR")
ner.add_label("DORMIR")
ner.add_label("GIRAR")
ner.add_label("LEVANTAR BRAÇOS")
ner.add_label("PIADA")
ner.add_label("FALAR O NOME")
ner.add_label("ESTÁ BEM")
ner.add_label("APRESENTAR")
ner.add_label("FALAR SOBRE SEMEAR")
ner.add_label("FALAR SOBRE ADA")
ner.add_label("MÚSICA")
ner.add_label("SOLETRAR")
ner.add_label("DANÇAR")

pipe_exceptions = ["ner", "trf_wordpiecer", "trf_tok2vec"]
unaffected_pipes = [pipe for pipe in nlp.pipe_names if pipe not in pipe_exceptions]

doc=nlp("Elon Musk, Bill Gates, Steve Jobs, 20 de Março, 2021")
for ent in doc.ents:
  print(ent.text,ent.label_)

#print(nlp.pipe_names)
doc = nlp("Acorda")
print("Comando", [(ent.text, ent.label_) for ent in doc.ents])

modelo.add_pipe(ner)
historico = []
"""