import queue
import random
import threading
from reconhecimentoAudio import reconhecerAudio
from reconhecimentoSentido import gerarResposta
from speech_recognition import UnknownValueError

class Lisa:
    lisas = {}

    def __init__(self):
        if len(Lisa.lisas) > 256:
            raise OverflowError("Lisas demais")

        self.pedidos   = queue.Queue(32)
        self.respostas = [None]*32

        uid = random.randint(0, int(1e11-1))
    
        #ineficiente 
        while uid in Lisa.lisas.keys():
            uid = random.randint(0, int(1e11-1))
        
        self.uid = uid
        Lisa.lisas[self.uid] = self
        
        threading.Thread(target=self.processarRespostas).start()


    def adicionarResposta(self, entrada, compreender=False):

        try:
            indice = next(filter(lambda res: res[1] == None, enumerate(self.respostas)))[0]
        except StopIteration:
            raise OverflowError("Respostas demais")
        
        if type(entrada) != str and type(entrada) != bytes:
            raise ValueError("Tipo inválido")

        #indica que o índice está sendo utilizado, mas não está pronto
        self.respostas[indice] = False

        self.pedidos.put((entrada, indice, compreender), block=False)

        return indice


    def processarRespostas(self):

        while True:
            pedido, indice, compreender = self.pedidos.get()

            if type(pedido) == str:
                texto = pedido
            elif type(pedido) == bytes:
                try:
                    texto = reconhecerAudio(pedido)
                except UnknownValueError:
                    texto = ""
            
            if not compreender:
                self.respostas[indice] = texto
                continue
            
            self.respostas[indice] = gerarResposta(texto)
