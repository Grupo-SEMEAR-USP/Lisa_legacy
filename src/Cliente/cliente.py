import requests
import playsound
import os
import sys
from gravarAudio import gravarAudioArquivo
from urllib.parse import urljoin
import time


class ClienteLisa:
    def __init__(self, url_base: str, debug: bool=True):
        self.debug = debug
        self.url = url_base

        if self.debug:
            self.informacao_transferida = 0
            print("Criando Lisa...")
        resposta = self.enviarHTTP(urljoin(self.url, "registrar"))

        self.uid = resposta.content.decode("utf-8")
        if self.debug:
            print(f"Lisa criada, uid {self.uid}")

    def __del__(self):
        if self.debug:
            print(f"\n\ninformação transferida:", end=" ")
            print(f"{self.informacao_transferida/1024**2} Mb")
    
    def responderTexto(self, texto, compreender: bool=True):
        if self.debug:
            print("enviando responder texto")
        
        url_pedido = urljoin(self.url, self.uid) + "/"
        url_pedido = urljoin(url_pedido, "responder")

        resposta = self.enviarHTTP(url_pedido, texto, headers={
            "content-type": "text/plain", 
            "compreender": "true" if compreender else "false"
        })

        return resposta.content.decode("utf-8")

    def responderAudio(self, nome_arquivo=None, compreender=True):
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
            "compreender": "true" if compreender else "false"
        })

        arq.close()
        
        if gravar_arquivo:
            os.remove(nome_arquivo)

        return resposta.content.decode("utf-8")
    
    def pegarResposta(self, indice, audio=False):
        url_pedido = urljoin(self.url, self.uid) + "/"
        url_pedido = urljoin(url_pedido, "respostas") + "/"
        url_pedido = urljoin(url_pedido, indice)
        resposta = self.enviarHTTP(url_pedido, headers={
            "accept": "audio/mp3" if audio else "text/plain"
        }, method="get")
        
        return resposta.content


    def falar(self, audio):
        if self.debug:
            print(f"falando áudio de tamanho {len(audio)//1024} kb")
        
        nome_arquivo = "tmp.wav"
        arq = open(nome_arquivo, "wb")
        arq.write(audio)
        arq.close()
        playsound.playsound(nome_arquivo)
        os.remove(nome_arquivo)
    
    def gravar(self, arq):
        if self.debug:
            print(f"gravando áudio em {arq}...")
        gravarAudioArquivo(arq)

    def enviarHTTP(self, url, content=None, headers=None, method="post"):
        t_ini = time.time()
        try:
            if method == "post":
                resposta = requests.post(url, content, headers=headers)
            elif method == "delete":
                resposta = requests.delete(url)
            elif method == "get":
                resposta = requests.get(url, content, headers=headers)
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
                print(resposta.content)
                print("-------------")

        if resposta.status_code >= 400:
            raise IOError("Retorno incorreto")
        return resposta
            
    

if __name__ == "__main__":
    try:
        lisa = ClienteLisa("http://localhost:8080", True)
    except IOError:
        print("Erro na criação da cliente")
        sys.exit(-1)

    try:
        while True:
            lido = input("input: ")
            try:
                comando = lido.split()[0]
            except IndexError:
                break
                
            try:
                if comando == "enviarAudio":
                    indice = lisa.responderAudio(None, compreender=False)
                    print(indice)
                elif comando == "enviarTexto":
                    texto = lido[lido.find(comando)+len(comando)+1:]
                    indice = lisa.responderTexto(texto, compreender=False)
                    print(indice)
                elif comando == "responderAudio":
                    indice = lisa.responderAudio(None)
                    print(indice)
                    lisa.falar(audio)
                elif comando == "responderTexto":
                    texto = lido[lido.find(comando)+len(comando)+1:]
                    indice = lisa.responderTexto(texto)
                    print(indice)
                elif comando == "pegarTexto":
                    indice = lido.split()[1]
                    texto = lisa.pegarResposta(indice, audio=False).decode("utf-8")
                    print(texto)
                elif comando == "pegarAudio":
                    indice = lido.split()[1]
                    audio = lisa.pegarResposta(indice, audio=True)
                    lisa.falar(audio)
                else:
                    print("Erro, input invalido")
            except IOError:
                continue
    except (EOFError, KeyboardInterrupt):
        pass