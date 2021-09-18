import spacy
import pandas as pd

train_data = pd.read_csv("Esqueleto\\frases_pln.csv", encoding = 'utf-8')
valid_data = pd.read_csv("Esqueleto\\frases_pln.csv", encoding = 'utf-8')
train_data=train_data.astype(str)
valid_data=valid_data.astype(str)

text = "Dormir"     #Digitar o comando para testar o modelo

nlp = spacy.load("Codigo\\Treinamentos_PLN\\IMDB\\output\\model-last")

doc = nlp(text[0])

print(doc.cats)     #Print da probabilidade de cada label

print(text)         #Print do comando digitado acima