import random
import structlog


logger = structlog.get_logger("Lisa")


class Lisa:
    '''
    A classe Lisa contém a interpretação interna do robô da Lisa.

    A classe em si tem a variável estática lisas, que contém todas as
    instâncias de clientes da Lisa.

    Cada instância contém uma fila de pedidos, uma lista de respostas e um uid.
    '''

    lisas = {}

    def __init__(self):
        #há um máximo de Lisas por questão de uso de memória
        if len(Lisa.lisas)+1 > 256:
            logger.error("Tentou criar mais Lisas que suportado", 
                stack_info=True)
            raise OverflowError("Lisas demais")
        if len(Lisa.lisas) == 255:
            logger.warning("Máximo de Lisas Atingido")

        #cria um uid aleatório de 10 casas decimais
        uid = random.randint(0, int(1e11-1))
    
        #não é o melhor caminho, mas é aceitável pois 256 << 1e11-1
        while uid in Lisa.lisas.keys():
            uid = random.randint(0, int(1e11-1))
        self.uid = uid

        #coloca essa Lisa na lista de Lisas
        logger.debug("Registrando Lisa", uid=self.uid, total=len(Lisa.lisas)+1)
        Lisa.lisas[self.uid] = self
    
    def responder(self, texto, compreender):
        if not compreender:
            return texto
        return "Eu não entendi"