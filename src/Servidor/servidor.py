import flask

from TTSLisa import gerarStreamAudio
from reconhecimentoSentido import gerarResposta, criarLisa
from reconhecimentoAudio import reconhecerAudio


#TODO:  tornar isso um REST api, se for possível


servidor = flask.Flask("servidorLisa")


@servidor.route("/registrar", methods=["POST"])
def registrar():
    
    uid = criarLisa()
    return flask.Response(status=201, headers={"uid":str(uid)})


@servidor.route("/para_audio", methods=["POST"])
def paraAudio():
    if flask.request.content_type != "text/plain":
        return flask.Response(status=400)
    
    texto = flask.request.get_data().decode("utf-8")

    gen_audio = None
    try:
        gen_audio = gerarStreamAudio(texto)
    except AssertionError:
        return flask.Response("No Text to Speak", status=400)

    return servidor.response_class(gen_audio(), mimetype="audio/mp3")


@servidor.route("/para_texto", methods=["POST"])
def paraTexto():
    if flask.request.content_type != "audio/wav":
        return flask.Response(status=400)
    
    audio = flask.request.get_data()
    texto = reconhecerAudio(audio)

    return flask.Response(texto, mimetype="text/plain")


@servidor.route("/responder/<uid>", methods=["GET"])
def responder(uid):
    
    try:
        uid = int(uid)
    except ValueError:
        return flask.Response("Bad uid", status=400)

    content_type = flask.request.content_type

    texto_in = None
    
    if content_type == "text/plain":
        texto_in = flask.request.get_data().decode("utf-8")
    elif content_type == "audio/wav":
        texto_in = reconhecerAudio(flask.request.get_data())
    else:
        return flask.Response(status=400)
    
    texto_out = None
    try:
        texto_out = gerarResposta(uid, texto_in)
    except KeyError:
        return flask.Response(status=404)
    
    gen_audio = None
    try:
        gen_audio = gerarStreamAudio(texto_out)
    except AssertionError:
        return flask.Response("No Text to Speak", status=400)

    return servidor.response_class(gen_audio(), mimetype="audio/mp3")


if __name__ == '__main__':

    servidor.run("0.0.0.0", 8080, True)
