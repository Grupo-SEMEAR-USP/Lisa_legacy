import requests
import playsound
import os
import sys
from gravarAudio import gravarAudioArquivo
from urllib.parse import urljoin
import time
import scipy.io.wavfile
import numpy as np


class ClienteLisa:
    '''
    A classe ClienteLisa é uma classe simples que define uma implementação
    pouco complexa de um cliente de comunicação, com métodos responderTexto e
    responderAudio para gerar a resposta da Lisa para um texto ou áudio,
    pegarResposta e deletarResposta para ver e deletar essas respostas e
    alguns métodos utilitátios
    '''

    def __init__(self, url_base, debug=False):
        self.debug = debug
        self.url = url_base

        if self.debug:
            self.informacao_transferida = 0
            print("Criando Lisa...")

        #registra a Lisa e guarda o UID
        resposta = self.enviarHTTP(urljoin(self.url, "registrar"))

        self.uid = resposta.content.decode("utf-8")
        if self.debug:
            print(f"Lisa criada, uid {self.uid}")

    def __del__(self):
        if self.debug:
            print(f"\n\ninformação transferida:", end=" ")
            print(f"{self.informacao_transferida/1024**2} Mb")

    def responderTexto(self, texto, accept, compreender):
        '''
        O método responderTexto envia um pedido em texto para o servidor da
        Lisa, compreendendo ou apenas copiando a resposta
        '''

        if self.debug:
            print("enviando responder texto")

        url_pedido = urljoin(self.url, self.uid) + "/"
        url_pedido = urljoin(url_pedido, "responder")

        resposta = self.enviarHTTP(url_pedido, texto, headers={
            "content-type": "text/plain",
            "accept": accept,
            "compreender": compreender
        })

        return resposta.content

    def responderAudio(self, accept, compreender, nome_arquivo=None):
        '''
        O método responderAudio faz o mesmo que responderTexto mas para um
        áudio, além de gravar um arquivo de áudio temporário do usuário caso
        não haja nenhum input de áudio prévio
        '''

        if self.debug:
            print(f"enviando responder texto")

        gravar_arquivo = nome_arquivo is None
        if gravar_arquivo:
            nome_arquivo = "tmp.wav"
            self.gravar(nome_arquivo)

        arq = open(nome_arquivo, "rb")


        url_pedido = urljoin(self.url, self.uid) + "/"
        url_pedido = urljoin(url_pedido, "responder")

        resposta = self.enviarHTTP(url_pedido, arq, headers={
            "content-type": "audio/wav",
            "accept": accept,
            "compreender": compreender
        })

        arq.close()

        if gravar_arquivo:
            os.remove(nome_arquivo)

        return resposta.content

    def falar(self, audio):
        '''
        O método falar fala um áudio contido em um objeto bytes
        '''

        if self.debug:
            print(f"falando áudio de tamanho {len(audio)//1024} kb")

        dados = np.frombuffer(audio, dtype=np.uint8)

        nome_arquivo = "tmp.wav"
        scipy.io.wavfile.write(nome_arquivo, 24000, dados)
        playsound.playsound(nome_arquivo)
        os.remove(nome_arquivo)

    def gravar(self, arq):
        '''
        O método gravar grava áudio do usuário em um arquivo arq
        '''

        if self.debug:
            print(f"gravando áudio em {arq}...")
        gravarAudioArquivo(arq)

    def enviarHTTP(self, url, content=None, headers=None):
        '''
        O método enviarHTTP envia um pedido de HTTP de forma uniforme com
        informações de debug úteis
        '''

        t_ini = time.time()
        try:
            resposta = requests.post(url, content, headers=headers)
        except requests.exceptions.ConnectionError:
            print(f"Erro! Não foi possível conectar a {url}")
            sys.exit(1)
        t_fim = time.time()

        if self.debug:
            self.informacao_transferida += len(resposta.content)

            print(f"retorno demorou {t_fim-t_ini}", end=" ")
            print(f"com código {resposta.status_code}", end=" ")
            try:
                print(f"com mimetype {resposta.headers['content-type']}")
            except KeyError:
                print("sem mimetype")

            if resposta.status_code >= 400:
                print("-------------")
                print("dump de erro:")
                print("-------------")
                print(resposta.headers)
                print("-------------")
                print(resposta.content.decode("utf-8"))
                print("-------------")

        if resposta.status_code >= 400:
            raise ConnectionError(resposta.content.decode("utf-8"))
        return resposta

ajuda = '''
Utilize "responder" seguido de três (ou quatro) argumentos:

o argumento 1 significa o tipo de input a ser enviado, podendo ser "texto" ou
"audio".

o argumento 2 significa o tipo de output a ser recebido, podendo ser "texto" ou
"audio".

o argumento 3 significa se o input deve ser compreendido de fato, ou somente
copiado do pedido (útil para, por exemplo, testar se a conexão com o servidor é
válida).

caso o input a ser enviado seja um texto, tudo após o terceiro argumento é
enviado para o servidor para ser interpretado.

exemplo: responder texto texto true oi lisa, tudo bem contigo?'''

if __name__ == "__main__":
    try:
        lisa = ClienteLisa("http://localhost:8080")
    except IOError:
        print("Erro na criação da cliente")
        sys.exit(-1)

    print("Programa de cliente teste da Lisa")
    print("Utilize \"ajuda\" para ver os comandos possíveis")

    try:
        while True:
            try:
                lido = input("input: ")
                comando = lido.split()
            except IndexError:
                break

            if len(comando) < 4 or comando[0] != "responder" or \
                (comando[1] not in ["texto", "audio"]) or \
                (comando[2] not in ["texto", "audio"]) or \
                (comando[3] not in ["true", "false"]):

                print(ajuda)
                continue

            content_type_texto = comando[1] == "texto"
            accept = "text/plain" if comando[2] == "texto" else "audio/wav"
            compreender = comando[3]

            if content_type_texto:
                if len(comando) < 5:
                    print(ajuda)
                    continue
                retorno = lisa.responderTexto(lido[lido.find("false")+6:],
                    accept, compreender)
            else:
                retorno = lisa.responderAudio(accept, compreender)

            if accept == "text/plain":
                print(retorno.decode("utf-8"))
                continue

            lisa.falar(retorno)

    except (EOFError, KeyboardInterrupt):
        pass
    except ConnectionError as e:
        print(f"erro no processamento do input: {e}")
