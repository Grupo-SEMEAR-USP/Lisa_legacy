from spacy.tokens import DocBin
from ml_datasets import imdb
import spacy

train_data, valid_data = imdb()

print(train_data[0])

def make_docs(data):
    docs = []
    for doc,label  in nlp.pipe(data, as_tuples=True):
        if label == "neg":
            doc.cats["positive"] = 0
            doc.cats["negative"] = 1
        else:
            doc.cats["positive"] = 1
            doc.cats["negative"] = 0
        docs.append(doc)
    return(docs)

nlp = spacy.load("en_core_web_sm")

num_texts = 500

train_docs = make_docs(train_data[:num_texts])
doc_bin = DocBin(docs=train_docs)
doc_bin.to_disk("./data/train.spacy")

valid_docs = make_docs(valid_data[:num_texts])
doc_bin = DocBin(docs=valid_docs)
doc_bin.to_disk("./data/valid.spacy")