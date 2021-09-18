'''
    O código abaixo foi inspirado no site:
        https://www.machinelearningplus.com/nlp/custom-text-classification-spacy/
    
    Problemas até o momento (18/09/21):
        - Está desatualizado, portanto algumas funções não funcionam

'''


import pandas as pd
import spacy
from spacy.pipeline.textcat import single_label_cnn_config

database = pd.read_csv("Esqueleto\\frases_pln.csv", encoding = 'utf-8')
#print(database.head())

nlp=spacy.load("pt_core_news_sm")
#print(nlp.pipe_names)

#Modelo do Spacy não possui textcat, então vamos add manualmente
nlp.add_pipe('textcat', config=single_label_cnn_config, last=True)
textcat = nlp.get_pipe('textcat')
print(nlp.pipe_names)