import pandas as pd
import string
import spacy
import random
import seaborn as sns
import numpy as np
import matplotlib.pyplot as plt
from spacy.language import Language
from spacy_langdetect  import LanguageDetector
from sklearn.feature_extraction.text import CountVectorizer

nlp = spacy.load("pt_core_news_sm")

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

baseDeDados['texto'] = baseDeDados['texto'].apply(preprocessamento)


#check do tratamento da base de dados
print(baseDeDados.head(10))
print(baseDeDados.tail())

cv = CountVectorizer(stop_words='english')
data_cv = cv.fit_transform(data_clean.transcript)
data_dtm = pd.DataFrame(data_cv.toarray(), columns=cv.get_feature_names())
data_dtm.index = data_clean.index
data_dtm 