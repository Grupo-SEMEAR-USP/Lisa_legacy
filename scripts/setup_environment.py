#!/usr/bin/env python3
import subprocess
import sys


#instalando todos os pacotes, não foi colocado em um arquivo .sh ou coisa do
#gênero para facilitar o uso para usuários de windows
ret = subprocess.call(["python3", "-m", "pip", "install",
                       "-r", "requirements.txt"])

if ret != 0: #em caso de erro
    print("erro na instalação de pacotes em requirements.txt", file=sys.stderr)
    sys.exit(-1)

#setup do spacy
ret = subprocess.call(["python3", "-m", "spacy", "download",
                       "pt_core_news_sm"])

if ret != 0: #em caso de erro
    print("erro na instalação de pt_core_news_sm", file=sys.stderr)

try:
    #setup do nltk
    import nltk
    nltk.download("punkt")
    nltk.download("stopwords")
    nltk.download("rslp")
except Exception as e:
    print(f"erro no setup do nltk, erro tipo {e.__class__}: {e}",
            file=sys.stderr)

