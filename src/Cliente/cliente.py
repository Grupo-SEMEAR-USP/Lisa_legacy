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

        self.uid = resposta.headers["uid"]
        if self.debug:
            print(f"Lisa criada, uid {self.uid}")

    def __del__(self):
        if self.debug:
            print(f"\n\ndeletando Lisa")
            print(f"informação transferida:", end=" ")
            print(f"{self.informacao_transferida/1024**2} Mb")
    
    def paraAudio(self, texto):
        if self.debug:
            print("enviando para_audio")
        resposta = self.enviarHTTP(urljoin(self.url, "para_audio"), texto, 
            headers={"content-type": "text/plain"})

        return resposta.content

    def paraTexto(self, nome_arquivo=None):
        if self.debug:
            print(f"enviando para_texto")

        gravar_arquivo = nome_arquivo is None
        if gravar_arquivo:
            nome_arquivo = "tmp.wav"
            self.gravar(nome_arquivo)
        
        arq = open(nome_arquivo, "rb")
            
        resposta = self.enviarHTTP(urljoin(self.url, "para_texto"), arq, 
            headers={"content-type": "audio/wav"})

        arq.close()
        
        if gravar_arquivo:
            os.remove(nome_arquivo)

        return resposta.content
    
    def responder(self, entrada, arquivo=False):
        if self.debug:
            print(f"enviando responder em {'audio' if arquivo else 'texto'}")

        mimetype = "text/plain" if not arquivo else "audio/wav"

        conteudo = open(entrada, "rb") if arquivo else entrada


        resposta = self.enviarHTTP(urljoin(self.url, f"responder/{self.uid}"),
            conteudo, headers={"content-type": mimetype})
        
        if arquivo:
            conteudo.close()

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

    def enviarHTTP(self, url, content=None, headers=None):
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
                print(resposta.content)
                print("-------------")
                sys.exit(1)
        
        return resposta
            
    

if __name__ == "__main__":
    lisa = ClienteLisa("http://localhost:8080", True)

    try:
        while True:
            lido = input("input: ")
            try:
                comando = lido.split()[0]
            except IndexError:
                break
                
            if comando == "paraAudio":
                texto = lido[lido.find(comando)+len(comando)+1:]
                audio = lisa.paraAudio(texto)
                lisa.falar(audio)
            elif comando == "paraTexto":
                texto = lisa.paraTexto(None)
                print(texto)
            elif comando == "responderAudio":
                lisa.gravar("tmp.wav")
                audio = lisa.responder("tmp.wav", True)
                os.remove("tmp.wav")
                lisa.falar(audio)
            elif comando == "responderTexto":
                texto = lido[lido.find(comando)+len(comando)+1:]
                audio = lisa.responder(texto)
                lisa.falar(audio)
            else:
                print("Erro, input invalido")

    except EOFError:
        pass
    except KeyboardInterrupt:
        pass