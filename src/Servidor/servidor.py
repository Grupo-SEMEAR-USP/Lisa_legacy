import flask
import structlog

from TTSLisa import gerarAudio
from reconhecimentoAudio import reconhecerAudio
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

def receberEntrada(pedido):
    '''
    A função receberEntrada apenas recebe, com base no header content_type,
    a entrada em texto de um pedido da função responder(). Caso o header
    content-type não esteja presente, ou caso ele não seja suportado, 
    ValueError e levantado. Caso a entrada seja um áudio, o reconhecimento de
    áudio é chamado para transformar a entrada em texto.
    '''

    if pedido.content_type == "":
        raise ValueError()
    
    if pedido.content_type == "text/plain":
        #pega o texto todo como uma string
        return pedido.get_data().decode("utf-8")

    if pedido.content_type == "audio/wav":
        #pega o áudio todo como uma sequência de bytes
        return reconhecerAudio(pedido.get_data())

    raise ValueError()


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
    A função responder retorna um áudio ou texto com uma resposta a um um áudio
    ou texto recebido. Essa é a função principal do servidor e é essencial para
    o funcionamento da robô.
    
    São necessários três headers: 
        compreender: pode ser true ou false para identificar se o valor deve
        ser interpretado ou somente repetido (útil para converter áudio em 
        texto e vice-versa)
    
        content-type: tipo de dado de entrada, pode ser text/plain ou 
        audio/wav.

        accept: tipo de dado aceito como saída, pode ser text/plain ou 
        audio/wav.
    '''
    
    #identificando o cliente
    try:
        lisa = Lisa.lisas[int(uid)]
    except KeyError:
        return respostaComLog("Lisa não encontrada", status=404)
    except ValueError:
        return respostaComLog("Uid não é um número", status=400)

    #identificando o header de compreensão
    try:
        compreender = not flask.request.headers["compreender"] == "false"
    except KeyError:
        compreender = True

    #identificando o tipo de entrada
    try:
        entrada = receberEntrada(flask.request)
    except UnicodeDecodeError:
        return respostaComLog("Entrada não é UTF-8", status=400)
    except ValueError:
        return respostaComLog(
            "header content-type não existe, não suportado ou incompatível "
            "com a entrada", status=400)
    except ConnectionError:
        return respostaComLog("Erro na conexão para reconhecimento de áudio",
            status=500)

    #identificando o tipo de dado pedido para retorno
    try:
        tipo_pedido = flask.request.headers["accept"]
    except KeyError:
        return respostaComLog("header accept não existe", status=400)


    #processando a entrada
    resposta = lisa.responder(entrada, compreender)

    #retornando a resposta com base no tipo pedido
    if tipo_pedido == "text/plain":
        #retornando o texto da resposta em si
        return respostaComLog(resposta, "Retornando texto")

    if tipo_pedido == "audio/wav":
        try:
            audio = gerarAudio(resposta)
        except AssertionError:
            return respostaComLog("Erro no TTS", status=500)

        return respostaComLog(audio.tobytes(), "Retornando áudio")
    
    #se não recebemos um accept válido retorna que não é suportado
    return respostaComLog(
        "header accept não suportado", status=400)


if __name__ == '__main__':
    #roda o servidor, em modo de debug, escutando na porta 8080 em todos os
    #hostnames, ou seja, é possível acessar o servidor em 
    #http://localhost:8080 ou http://seu_ip_local:8080, por exemplo

    servidor.run("0.0.0.0", 8080, True)
