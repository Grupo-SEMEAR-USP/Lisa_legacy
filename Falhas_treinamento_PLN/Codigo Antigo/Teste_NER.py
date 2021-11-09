import pandas as pd
import string
import spacy
import random
import seaborn as sns
import numpy as np
import matplotlib.pyplot as plt
from spacy.language import Language
from spacy_langdetect  import LanguageDetector

def get_lang_detector(nlp, name):
  return LanguageDetector()

nlp = spacy.load("pt_core_news_sm")
print(nlp.pipe_names)
#check do carregamento
#print(pln)

#carregamento da base de dados
#sheet_ID = '1lKD0M7h1Knaer2nETaVdM1RF80PMx-3Rsorb86NNVO0'
#baseDeDados = pd.read_csv(f"https://docs.google.com/spreadsheets/d/{sheet_ID}/export?format=csv")
baseDeDados = pd.read_csv("DataBaseFinal.csv", encoding = 'utf-8')
baseDeDados = baseDeDados.astype(str)


for _, annotations in baseDeDados:
  for ent in annotations.get("Comando"):
    ner.add_label(ent[2])

pipe_exceptions = ["ner"]
unaffected_pipes = [pipe for pipe in nlp.pipe_names if pipe not in pipe_exceptions]

doc=nlp("Elon Musk, Bill Gates, Steve Jobs, 20 de Março, 2021")
for ent in doc.ents:
  print(ent.text,ent.label_)

