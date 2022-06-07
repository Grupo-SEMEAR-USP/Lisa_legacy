import flask

from TTSLisa import gerarStreamAudio
from reconhecimentoSentido import gerarResposta, criarLisa
from reconhecimentoAudio import reconhecerAudio
from speech_recognition import UnknownValueError


#TODO:  tornar isso um REST api, se for possível


servidor = flask.Flask("servidorLisa")


@servidor.route("/registrar", methods=["POST"])
def registrar():
    '''
    A função registrar registra uma nova instância de Lisa, 
    retornando um valor do ID do cliente.
    '''

    try:
        uid = criarLisa()
    except OverflowError:
        flask.Response("Lisas demais", status=507)
    
    return flask.Response(status=201, headers={"uid":str(uid)})


@servidor.route("/para_audio", methods=["POST"])
def paraAudio():
    '''
    A função paraAudio transforma o texto recebido no corpo do pedido em um 
    áudio, feito para facilitar debugging de comunicação.
    '''

    #se não recebems texto algo está errado
    if flask.request.content_type != "text/plain":
        return flask.Response(status=400)
    
    #pega o texto todo como uma string
    texto = flask.request.get_data().decode("utf-8")

    try:
        gerador = gerarStreamAudio(texto)
    except AssertionError:
        #se o input é vazio ou incorreto, falha
        return flask.Response("No Text to Speak", status=400)

    #retorna áudio via streaming
    return servidor.response_class(gerador(), mimetype="audio/mp3")


@servidor.route("/para_texto", methods=["POST"])
def paraTexto():
    '''
    A função paraTexto transforma um audio wav recebido em texto, 
    feito para facilitar debugging de comunicação.
    '''

    #se não recebemos um wav algo está errado
    if flask.request.content_type != "audio/wav":
        return flask.Response(status=400)
    
    audio = flask.request.get_data()

    try:
        texto = reconhecerAudio(audio)
    except UnknownValueError:
        return flask.Response("No text to Speak", status=400)

    return flask.Response(texto, mimetype="text/plain")


@servidor.route("/responder/<uid>", methods=["POST"])
def responder(uid):
    '''
    A função responder retorna o áudio de TTS com a resposta da Lisa da 
    instância uid para a fala recebida no corpo do pedido. Essa é a função 
    principal do servidor e é essencial para o funcionamento da robô.
    '''
    
    try:
        uid = int(uid)
    except ValueError:
        #se o uid não é um inteiro, falha
        return flask.Response("Bad uid", status=400)

    content_type = flask.request.content_type
    

    #são aceitos tanto texto quanto áudio, agindo apropriadamente para cada um 
    #e reconhecendo o tipo de input via o header de http "content-type",
    #também conhecido como "mimetype"
    if content_type == "text/plain":
        texto_in = flask.request.get_data().decode("utf-8")
    elif content_type == "audio/wav":
        try:
            texto_in = reconhecerAudio(flask.request.get_data())
        except UnknownValueError:
            texto_in = None
    else:
        return flask.Response(status=400)
    
    try:
        texto_out = gerarResposta(uid, texto_in)
    except KeyError:
        #se essa Lisa não existe, retorna que não encontrada
        return flask.Response(status=404)
    
    try:
        gerador = gerarStreamAudio(texto_out)
    except AssertionError:
        #se a Lisa não conseguiu responder, ocorreu um erro em gerarResposta!
        return flask.Response("Lisa Could not answer", status=500)

    #retorna áudio de resposta via streaming
    return servidor.response_class(gerador(), mimetype="audio/mp3")


if __name__ == '__main__':
    #roda o servidor, em modo de debug, escutando na porta 8080 em todos os
    #hostnames, ou seja, é possível acessar o servidor em 
    #http://localhost:8080 ou http://192.168.0.seu_ip:8080, por exemplo

    servidor.run("0.0.0.0", 8080, True)
