'''
    O código abaixo foi inspirado nas explicações do site:
        https://www.youtube.com/watch?v=7PD48PFL9VQ
        GitHub -> https://github.com/wjbmattingly/youtube_text_classification
    
    Problemas encontrados até o momento (18/09/21):
        - Não sabemos como funciona a matriz textcat e nem token2vec para vermos se está sendo criada corretamente
        - Após testarmos, independente do texto analisado, sempre mostra a mesma probabilidade de cada label
'''

from spacy.tokens import DocBin
import pandas as pd
import spacy
import csv
import pandas as pd

train_data = pd.read_csv("Codigo\\Treinamentos_PLN\\IMDB\\frases_pln.csv", encoding = 'utf-8')
valid_data = pd.read_csv("Codigo\\Treinamentos_PLN\\IMDB\\frases_pln.csv", encoding = 'utf-8')
train_data=train_data.astype(str)
valid_data=valid_data.astype(str)
print(train_data.head())

nlp = spacy.load("pt_core_news_sm")

num_texts = 855     #Número de frases que código vai analisar

def make_docs(data):
    docs = []
    for doc,label  in nlp.pipe(zip(data['Frase'],data['Comando']), as_tuples=True):
        if label == "Acordar":
            doc.cats["Dancar"] = 0
            doc.cats["Acordar"] = 1
            doc.cats["Dormir"] = 0
            doc.cats["Girar"] = 0
            doc.cats["Levantar_bracos"] = 0
            doc.cats["Piada"] = 0
            doc.cats["Esta_bem"] = 0
            doc.cats["Apresentar"] = 0
            doc.cats["SEMEAR"] = 0
            doc.cats["ADA"] = 0
            doc.cats["Musica"] = 0
            doc.cats["Soletrar"] = 0
        elif label=="Dormir":
            doc.cats["Dancar"] = 0
            doc.cats["Acordar"] = 0
            doc.cats["Dormir"] = 1
            doc.cats["Girar"] = 0
            doc.cats["Levantar_bracos"] = 0
            doc.cats["Piada"] = 0
            doc.cats["Esta_bem"] = 0
            doc.cats["Apresentar"] = 0
            doc.cats["SEMEAR"] = 0
            doc.cats["ADA"] = 0
            doc.cats["Musica"] = 0
            doc.cats["Soletrar"] = 0
        elif label=="Girar":
            doc.cats["Dancar"] = 0
            doc.cats["Acordar"] = 0
            doc.cats["Dormir"] = 0
            doc.cats["Girar"] = 1
            doc.cats["Levantar_bracos"] = 0
            doc.cats["Piada"] = 0
            doc.cats["Esta_bem"] = 0
            doc.cats["Apresentar"] = 0
            doc.cats["SEMEAR"] = 0
            doc.cats["ADA"] = 0
            doc.cats["Musica"] = 0
            doc.cats["Soletrar"] = 0
        elif label=="Dançar":
            doc.cats["Dancar"] = 1
            doc.cats["Acordar"] = 0
            doc.cats["Dormir"] = 0
            doc.cats["Girar"] = 0
            doc.cats["Levantar_bracos"] = 0
            doc.cats["Piada"] = 0
            doc.cats["Esta_bem"] = 0
            doc.cats["Apresentar"] = 0
            doc.cats["SEMEAR"] = 0
            doc.cats["ADA"] = 0
            doc.cats["Musica"] = 0
            doc.cats["Soletrar"] = 0
        elif label=="Levantar braços":
            doc.cats["Dancar"] = 0
            doc.cats["Acordar"] = 0
            doc.cats["Dormir"] = 0
            doc.cats["Girar"] = 0
            doc.cats["Levantar_bracos"] = 1
            doc.cats["Piada"] = 0
            doc.cats["Esta_bem"] = 0
            doc.cats["Apresentar"] = 0
            doc.cats["SEMEAR"] = 0
            doc.cats["ADA"] = 0
            doc.cats["Musica"] = 0
            doc.cats["Soletrar"] = 0
        elif label=="Piada":
            doc.cats["Dancar"] = 0
            doc.cats["Acordar"] = 0
            doc.cats["Dormir"] = 0
            doc.cats["Girar"] = 0
            doc.cats["Levantar_bracos"] = 0
            doc.cats["Piada"] = 1
            doc.cats["Esta_bem"] = 0
            doc.cats["Apresentar"] = 0
            doc.cats["SEMEAR"] = 0
            doc.cats["ADA"] = 0
            doc.cats["Musica"] = 0
            doc.cats["Soletrar"] = 0
        elif label=="Está bem":
            doc.cats["Dancar"] = 0
            doc.cats["Acordar"] = 0
            doc.cats["Dormir"] = 0
            doc.cats["Girar"] = 0
            doc.cats["Levantar_bracos"] = 0
            doc.cats["Piada"] = 0
            doc.cats["Esta_bem"] = 1
            doc.cats["Apresentar"] = 0
            doc.cats["SEMEAR"] = 0
            doc.cats["ADA"] = 0
            doc.cats["Musica"] = 0
            doc.cats["Soletrar"] = 0
        elif label=="Falar o nome" or label=="Apresentar":
            doc.cats["Dancar"] = 0
            doc.cats["Acordar"] = 0
            doc.cats["Dormir"] = 0
            doc.cats["Girar"] = 0
            doc.cats["Levantar_bracos"] = 0
            doc.cats["Piada"] = 0
            doc.cats["Esta_bem"] = 0
            doc.cats["Apresentar"] = 1
            doc.cats["SEMEAR"] = 0
            doc.cats["ADA"] = 0
            doc.cats["Musica"] = 0
            doc.cats["Soletrar"] = 0
        elif label=="Falar sobre SEMEAR":
            doc.cats["Dancar"] = 0
            doc.cats["Acordar"] = 0
            doc.cats["Dormir"] = 0
            doc.cats["Girar"] = 0
            doc.cats["Levantar_bracos"] = 0
            doc.cats["Piada"] = 0
            doc.cats["Esta_bem"] = 0
            doc.cats["Apresentar"] = 0
            doc.cats["SEMEAR"] = 1
            doc.cats["ADA"] = 0
            doc.cats["Musica"] = 0
            doc.cats["Soletrar"] = 0
        elif label=="Falar sobre ADA":
            doc.cats["Dancar"] = 0
            doc.cats["Acordar"] = 0
            doc.cats["Dormir"] = 0
            doc.cats["Girar"] = 0
            doc.cats["Levantar_bracos"] = 0
            doc.cats["Piada"] = 0
            doc.cats["Esta_bem"] = 0
            doc.cats["Apresentar"] = 0
            doc.cats["SEMEAR"] = 0
            doc.cats["ADA"] = 1
            doc.cats["Musica"] = 0
            doc.cats["Soletrar"] = 0
        elif label=="Soletrar":
            doc.cats["Dancar"] = 0
            doc.cats["Acordar"] = 0
            doc.cats["Dormir"] = 0
            doc.cats["Girar"] = 0
            doc.cats["Levantar_bracos"] = 0
            doc.cats["Piada"] = 0
            doc.cats["Esta_bem"] = 0
            doc.cats["Apresentar"] = 0
            doc.cats["SEMEAR"] = 0
            doc.cats["ADA"] = 0
            doc.cats["Musica"] = 0
            doc.cats["Soletrar"] = 1
        elif label=="Musica":
            doc.cats["Dancar"] = 0
            doc.cats["Acordar"] = 0
            doc.cats["Dormir"] = 0
            doc.cats["Girar"] = 0
            doc.cats["Levantar_bracos"] = 0
            doc.cats["Piada"] = 0
            doc.cats["Esta_bem"] = 0
            doc.cats["Apresentar"] = 0
            doc.cats["SEMEAR"] = 0
            doc.cats["ADA"] = 0
            doc.cats["Musica"] = 1
            doc.cats["Soletrar"] = 0
        docs.append(doc)        #Coloca os itens em uma lista
        #print(doc)
    return(docs)

#train_docs = make_docs(train_data[:num_texts])

train_docs = make_docs(train_data[:num_texts])
doc_bin = DocBin(docs=train_docs)
doc_bin.to_disk("Codigo\\Treinamentos_PLN\\IMDB\\train.spacy")

valid_docs = make_docs(valid_data[:num_texts])
doc_bin = DocBin(docs=valid_docs)
doc_bin.to_disk("Codigo\\Treinamentos_PLN\\IMDB\\valid.spacy")
