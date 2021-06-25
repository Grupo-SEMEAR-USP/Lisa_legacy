from spacy.tokens import DocBin
import pandas as pd
import spacy

train_data, valid_data = pd.read_csv("frases_pln.csv")

print(train_data[0])

def make_docs(data):
    docs = []
    for doc,label  in nlp.pipe(data, as_tuples=True):
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

        if label == "Acordar":
            doc.cats["Acordar"] = 1
        elif label=="Dormir":
            doc.cats["Dormir"] = 1
        elif label=="Girar":
            doc.cats["Girar"]=1
        elif label=="Dançar":
            doc.cats["Dançar"]=1
        elif label=="Levantar braços":
            doc.cats["Levantar_bracos"]=1
        elif label=="Piada":
            doc.cats["Piada"]=1
        elif label=="Está bem":
            doc.cats["Esta_bem"]=1
        elif label=="Falar o nome" or label=="Apresentar":
            doc.cats["Apresentar"]=1
        elif label=="Falar sobre SEMEAR":
            doc.cats["SEMEAR"]=1
        elif label=="Falar sobre ADA":
            doc.cats["ADA"]=1
        elif label=="Soletrar":
            doc.cats["Soletrar"]=1
        elif label=="Musica":
            doc.cats["Musica"]=1
        docs.append(doc)
    return(docs)

nlp = spacy.load("pt_core_news_lg")

num_texts = 855

train_docs = make_docs(train_data[:num_texts])
doc_bin = DocBin(docs=train_docs)
doc_bin.to_disk("./Dados/train.spacy")

valid_docs = make_docs(valid_data[:num_texts])
doc_bin = DocBin(docs=valid_docs)
doc_bin.to_disk("./Dados/valid.spacy")