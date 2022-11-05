import random
import structlog
import os
import sys
import ibm_watson
import ibm_cloud_sdk_core
from ibm_cloud_sdk_core.authenticators import IAMAuthenticator


logger = structlog.get_logger("Lisa")


class Lisa:
    '''
    A classe Lisa contém a interpretação interna do robô da Lisa.

    A classe em si tem a variável estática lisas, que contém todas as
    instâncias de clientes da Lisa.
    '''

    lisas = {}
    id_assistente = None
    assistente = None

    @staticmethod
    def setup_api_watson():
        logger.info("Inicializando api do IBM Watson")
        
        try:
            chave_api_watson = os.environ["API_KEY"]
            url_watson = os.environ["API_URL"]
            Lisa.id_assistente = os.environ["ASSISTANT_ID"]
        except KeyError:
            logger.critical("Variáveis de ambiente não settadas")
            sys.exit(-1)

        autenticador = IAMAuthenticator(chave_api_watson)
        Lisa.assistente = ibm_watson.AssistantV2('2021-06-14', autenticador)
        
        Lisa.assistente.set_service_url(url_watson)

    def __init__(self):
        #se não foram definidas as variáveis do api, dá pânico
        if Lisa.id_assistente is None:
            logger.critical("Tentou criar Lisa sem inicializar API")
            sys.exit(-1)

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

        #cria a sessão do IBM Watson
        try:
            retorno = Lisa.assistente.create_session(Lisa.id_assistente)
        except ibm_cloud_sdk_core.api_exception.ApiException:
            raise ConnectionError()
        
        if retorno.get_status_code() >= 400:
            logger.error("Erro criando sessão", 
                status=retorno.get_status_code(), saida=retorno.get_result(),
                id=self.uid)
            raise ConnectionError()
        
        self.id_sessao = retorno.get_result()["session_id"]

        Lisa.lisas[self.uid] = self
    
    def __del__(self):
        try:
            Lisa.lisas.pop(self.uid)
        except (KeyError, AttributeError):
            return #não conseguiu se registrar (deu um erro no __init__), ignora

        retorno = Lisa.assistente.delete_session(Lisa.id_assistente, 
            self.id_sessao)
        
        if retorno.get_status_code() >= 400:
            logger.error("Erro deletando sessão", 
                status=retorno.get_status_code(), saida=retorno.get_result())

    def responder(self, texto, compreender):
        if not compreender:
            if texto == "":
                return "Eu não entendi"
            return texto
        
        logger.info("Enviando mensagem para api do IBM Watson", mensagem=texto)
        retorno = Lisa.assistente.message(
            Lisa.id_assistente,
            self.id_sessao,
            input={'text': texto}
        ).get_result()

        logger.debug("Rebendo resposta do api do IBM Watson", resposta=retorno)

        return retorno["output"]["generic"][0]["text"]