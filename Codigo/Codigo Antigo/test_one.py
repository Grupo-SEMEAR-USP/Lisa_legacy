import spacy
from ml_datasets import imdb

train_data, valid_data = imdb()

text = "Sebastian Roché? Why does he have first billing? His acting is terrible, he whines and shrinks and shrivels at every turn. He is not believable as someone a professional killer would save. The actress who leads this film is very good. She is an ex-British agent of some kind on the run from Columbians, she encounters the inspid, timid, unredeemable man ( Sebastian Roché) early one through a photo that he takes. The action here is slow, the direction is not good at all, the story is good but could have been, should have been much better. The photography does a pretty poor job of taking in the spectacular vistas of the great basin desert. the woman here is the story and she gets second billing, a fact that is absolutely insulting and outrageous. The so-called male lead is not worth the film he used up being on screen. It is to bad, with a few tweaks by better professionals, this could have been a much better film."

nlp = spacy.load("output/model-last")

doc = nlp(text[0])

print(doc.cats)

print(text)