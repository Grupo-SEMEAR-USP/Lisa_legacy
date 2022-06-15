import flask
import queue
import structlog

from TTSLisa import gerarStreamAudio
from reconhecimentoSentido import gerarResposta
from reconhecimentoAudio import reconhecerAudio
from speech_recognition import UnknownValueError
from Lisa import Lisa


servidor = flask.Flask("flask")
logger = structlog.get_logger("servidorLisa")


def respostaComLog(mensagem, status, *args):
    '''
    A função respostaComLog retorna uma resposta de flask, além de loggar
    eventos com o logger no nível debug caso o status não seja um erro do
    servidor e no nível debug caso seja

    Essa função deve ser utilizada quando se intende que o retorno seja uma
    informação de erro de alguma forma, pois o conteúdo da mensagem é utilizado
    como a mensagem de logging.
    '''

    if status < 500:
        logger.debug(
            mensagem, status=status, 
            caminho=flask.request.path,
            metodo=flask.request.method
        )
    else:
        #como o caminho e método pode ser diretamente manipulado pelo usuário,
        #não podemos incluir em logs que não sejam de debug por segurança
        logger.error(mensagem, status=status)
    
    return flask.Response(mensagem, status, *args)


@servidor.route("/registrar", methods=["POST"])
def registrar():
    '''
    A função registrar registra uma nova instância de Lisa, 
    retornando um valor do ID do cliente.
    '''

    try:
        lisa = Lisa()
    except OverflowError:
        return respostaComLog("Lisas demais", status=507)
    
    return flask.Response(str(lisa.uid))


@servidor.route("/<uid>/responder", methods=["POST"])
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
    return flask.Response(str(indice_pedido))


@servidor.route("/<uid>/respostas/<indice>", methods=["GET", "DELETE"])
def resposta(uid, indice):
    '''
    A função resposta retorna a resposta em áudio ou texto com um índice
    específico, ou deleta a resposta caso o metodo seja esse

    É necessária a presença de um header "accept" que identifica o tipo de dado
    pedido, sendo um de "text/plain" ou "audio/mp3"
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
        return flask.Response(status=200)


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
        return flask.Response(resposta)

    if tipo_pedido == "audio/mp3":
        #gerando a stream de áudio de TTS
        try:
            gerador = gerarStreamAudio(resposta)
        except AssertionError:
            return respostaComLog("Erro no TTS", status=500)
        
        #retornando a stream de áudio
        return servidor.response_class(gerador())
    

    #se não recebemos o accept correto retorna que não é suportado
    return respostaComLog("Tipo de dado não suportado", status=415)


if __name__ == '__main__':
    #roda o servidor, em modo de debug, escutando na porta 8080 em todos os
    #hostnames, ou seja, é possível acessar o servidor em 
    #http://localhost:8080 ou http://192.168.0.seu_ip:8080, por exemplo

    servidor.run("0.0.0.0", 8080, True)
