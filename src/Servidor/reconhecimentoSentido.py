import random


#TODO:  implementar falas em json
#       implementar UIDs corretamente


class Lisa:
    def __init__(self):
        self.uid = random.randint(0, 256)

lisas = {}

def criarLisa():

    nova_lisa = Lisa()    

    lisas[nova_lisa.uid] = nova_lisa
    return nova_lisa.uid

def gerarResposta(uid, texto_in):

    lisa = lisas[uid]

    return "Eu não entendi"