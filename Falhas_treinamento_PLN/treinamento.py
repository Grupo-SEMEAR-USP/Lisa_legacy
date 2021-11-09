
import spacy

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
nlp = spacy.load("pt_core_news_sm")
print(nlp.pipe_names)


nlp.initialize()

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