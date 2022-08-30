import flask
import queue
import structlog

from TTSLisa import gerarAudio
from reconhecimentoSentido import gerarResposta
from reconhecimentoAudio import reconhecerAudio
from speech_recognition import UnknownValueError
from Lisa import Lisa


servidor = flask.Flask("flask")
logger = structlog.get_logger("servidorLisa")


def routeComLog(caminho, **opcoes):
    '''
    A função routeComLog retorna um decorador que adiciona uma função à uma
    rota de caminho de HTTP para o servidor, adicionando uma mensagem de log
    para cada pedido que ocorrer. Os argumentos são os mesmos que para o
    método flask.Flask.route, pois eles realizam a mesma coisa só que
    precedendo do log.
    '''

    #cria o decorador em si
    def decorador(funcao):
        #cria a nova função que será retornada contendo o log e a chamada da 
        #função original
        def funcao_nova(*args, **kwargs):
            logger.info("Recebendo pedido HTTP", 
                caminho=flask.request.path, 
                metodo=flask.request.method,
                endereco_remoto=flask.request.remote_addr
            )
            return funcao(*args, **kwargs)

        servidor.add_url_rule(caminho, caminho, funcao_nova, **opcoes)
        return funcao_nova

    return decorador

def respostaComLog(mensagem, log=None, status=200, *args):
    '''
    A função respostaComLog retorna uma resposta de flask, além de loggar
    eventos com logs de debug em respostas ok ou de procedimento, info em
    respostas com erro de usuário e erro em respostas com erro de servidor
    '''

    if status < 400:
        funcao_logging = logger.debug
    elif status < 500:
        funcao_logging = logger.info
    else:
        funcao_logging = logger.error
    
    funcao_logging(log if log is not None else mensagem, 
        *args,
        status=status,
        caminho=flask.request.path,
        metodo=flask.request.method,
        endereco_remoto=flask.request.remote_addr
    )
    
    return flask.Response(mensagem, status, *args)


@routeComLog("/registrar", methods=["POST"])
def registrar():
    '''
    A função registrar registra uma nova instância de Lisa, 
    retornando um valor do ID do cliente.
    '''

    try:
        lisa = Lisa()
    except OverflowError:
        return respostaComLog("Lisas demais", status=507)
    
    return respostaComLog(str(lisa.uid), "Retornando registro de Lisa")

@routeComLog("/<uid>/responder", methods=["POST"])
def responder(uid):
    '''
    A função responder cria uma entrada nas respostas com o texto da resposta
    da Lisa a um áudio ou texto recebido no corpo do pedido. Essa é a função 
    principal do servidor e é essencial para o funcionamento da robô.
    
    Caso o header "compreender" esteja presente e tenha valor "false", o texto
    não passará pelo processamento de sentido, sendo apenas copiado para a
    lista de respostas, feito para debugging do código de comunicação

    É necessário para identificar o tipo de entrada a presença do header
    "content-type", contendo um de "text/plain" ou "audio/wav"
    '''
    
    #identificando o cliente
    try:
        lisa = Lisa.lisas[int(uid)]
    except KeyError:
        return respostaComLog("Lisa não encontrada", status=404)
    except ValueError:
        return respostaComLog("Uid não é um número", status=400)

    #identificando o tipo de entrada
    if flask.request.content_type == "text/plain":
        #pega o texto todo como uma string
        try:
            entrada = flask.request.get_data().decode("utf-8")
        except UnicodeDecodeError:
            return respostaComLog("Entrada não é UTF-8", status=400)

    elif flask.request.content_type == "audio/wav":
        #pega o áudio todo como uma sequência de bytes
        entrada = flask.request.get_data()
    
    else:
        #se não recebems texto ou wav algo está errado
        return respostaComLog("Tipo de dado não suportado", status=415)


    #identificando o header de compreensão
    try:
        compreender = not flask.request.headers["compreender"] == "false"
    except KeyError:
        compreender = True

    #processando a entrada
    try:
        #pede para a Lisa processar essa entrada
        indice_pedido = lisa.adicionarPedido(entrada, compreender)
    except queue.Full:
        return respostaComLog("Muitos pedidos no buffer", status=507)
    except OverflowError:
        return respostaComLog("Respostas demais", status=507)

    #retornando o índice que a resposta se encontrará na lista
    return respostaComLog(str(indice_pedido), "Retornando indice")


@routeComLog("/<uid>/respostas/<indice>", methods=["GET", "DELETE"])
def resposta(uid, indice):
    '''
    A função resposta retorna a resposta em áudio ou texto com um índice
    específico, ou deleta a resposta caso o metodo seja esse

    É necessária a presença de um header "accept" que identifica o tipo de dado
    pedido, sendo um de "text/plain" ou "audio/wav"
    '''

    #identificando o cliente
    try:
        lisa = Lisa.lisas[int(uid)]
    except KeyError:
        return respostaComLog("Lisa não encontrada", status=404)
    except ValueError:
        return respostaComLog("Uid não é um número", status=400)


    #identificando a resposta baseado no índice
    try:
        resposta = lisa.respostas[int(indice)]
    except ValueError:
        return respostaComLog("Indice não é um número", status=400)
    except KeyError:
        return respostaComLog("Indice não está entre 0 e 32", status=400)


    #deletando a resposta caso o metodo seja esse
    if flask.request.method == "DELETE":
        lisa.respostas[int(indice)] = None
        return respostaComLog("ok", "Retornando ok para o delete de Lisa")


    #identificando se a resposta está pronta e existe
    if type(resposta) == bool and resposta == False:
        return respostaComLog("Resposta não está pronta", status=202)
    if type(resposta) == type(None):
        return respostaComLog("Resposta não encontrada", status=404)
    

    #identificando o tipo de dado pedido para retorno
    try:
        tipo_pedido = flask.request.headers["accept"]
    except KeyError:
        return respostaComLog("Sem header Accept", status=400)

    if tipo_pedido == "text/plain":
        #retornando o texto da resposta em si
        return respostaComLog(resposta, "Retornando texto")

    if tipo_pedido == "audio/wav":
        try:
            audio = gerarAudio(resposta)
        except AssertionError:
            return respostaComLog("Erro no TTS", status=500)

        return respostaComLog(audio.tobytes(), "Retornando áudio")
    

    #se não recebemos o accept correto retorna que não é suportado
    return respostaComLog("Tipo de dado não suportado", status=415)


if __name__ == '__main__':
    #roda o servidor, em modo de debug, escutando na porta 8080 em todos os
    #hostnames, ou seja, é possível acessar o servidor em 
    #http://localhost:8080 ou http://seu_ip_local:8080, por exemplo

    servidor.run("0.0.0.0", 8080, True)
