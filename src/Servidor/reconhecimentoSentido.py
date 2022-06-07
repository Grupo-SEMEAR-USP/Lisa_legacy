import random


#TODO:  implementar falas em json
#       implementar UIDs corretamente


class Lisa:
    def __init__(self, uid):
        self.uid = uid

lisas = {}

def criarLisa():
    if len(lisas) > 256:
        raise OverflowError("Lisas demais")

    uid = random.randint(0, 2**32-1)
    
    #ineficiente 
    while uid in lisas.keys():
        uid = random.randint(0, 2**32-1)

    nova_lisa = Lisa(uid)

    lisas[nova_lisa.uid] = nova_lisa
    return nova_lisa.uid

def gerarResposta(uid, texto_in):

    lisa = lisas[uid]

    return "Eu não entendi"