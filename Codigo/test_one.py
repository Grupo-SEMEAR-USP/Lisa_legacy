import spacy
from ml_datasets import imdb

train_data, valid_data = imdb()

text = "this movie was fantastic"

nlp = spacy.load("output/model-last")

doc = nlp(text[0])

print(doc.cats)

print(text)