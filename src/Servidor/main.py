#!/usr/bin/env python3
import structlog
import logging
import waitress
import argparse
import sys

from servidor import servidor


nivel_logs_dict = {
    "debug":logging.DEBUG, 
    "info":logging.INFO, 
    "warning":logging.WARNING, 
    "error":logging.ERROR, 
    "critical":logging.CRITICAL
}

def tirarUnderscores(logger, log_method, event_dict):
    '''
    A função tirarUnderscores é um processador de structlog que retira todas
    as chaves no dicionário de eventos que começam com "_", isso foi feito pois
    utilizar o structlog.stdlib.ProcessorFormatter cria chaves extras com "_" 
    que não tem significado para o logging e apenas criam informações inúteis
    '''
    
    underscores = [k for k in event_dict.keys() if k[0]=="_"]
    for k in underscores:
        event_dict.pop(k)
    return event_dict


def configurarLogging(nivel_logs, json=False):
    '''
    A função configurarLogging configura o structlog para todo o programa da 
    Lisa, utilizando o nível de logging especificado, além de opcionalmente 
    utilizar JSON como saída, diferente da formatação para terminal padrão.
    '''
    
    #renderizador final que vai colocar na tela a saída dos logs
    if json:
        renderizador = structlog.processors.JSONRenderer()
    else:
        renderizador = structlog.dev.ConsoleRenderer()

    #apenas pede para que todos os logs do structlog passem pelo formatador
    structlog.configure(
        processors=[
            structlog.stdlib.ProcessorFormatter.wrap_for_formatter,
        ],
        logger_factory=structlog.stdlib.LoggerFactory(),
    )


    #formatador que deixa as nossas mensagens bem organizadas, tendo desde o
    #nome do logger utilizado até a hora que o log ocorreu e traces de stack
    #caso seja pedido
    formatador = structlog.stdlib.ProcessorFormatter(
        processors=[
            structlog.stdlib.add_logger_name,
            structlog.stdlib.add_log_level,
            tirarUnderscores,
            structlog.processors.format_exc_info,
            structlog.processors.StackInfoRenderer(),
            structlog.processors.TimeStamper("%X", utc=False),
            renderizador
        ],
    )

    #executor que vai conectar todos os logs do módulo logging ao formatador
    executor_logs = logging.StreamHandler()
    executor_logs.setFormatter(formatador)

    #pega o logger que baseia todos os outros (root) e configura
    logger_base = logging.getLogger()
    logger_base.addHandler(executor_logs)
    logger_base.setLevel(nivel_logs)


def lerArgumentos():
    '''
    A função lerArgumentos define e lê todos os argumentos de linha de comando
    retornando todos os argumentos
    '''

    leitor_args = argparse.ArgumentParser(
        description="Programa inicial do servidor da Lisa",
        add_help=False
    )

    leitor_args.add_argument(
        "-h", "--help", action="store_true",
        help="Mostra essa mensagem de ajuda e sai do programa"
    )
    leitor_args.add_argument(
        "-j", "--json", action="store_true",
        help="Realiza logs por JSON ao invés da interface de terminal"
    )
    leitor_args.add_argument(
        "-n", "--nivel", default="info", choices=nivel_logs_dict.keys(),
        help="Coloca na tela todos os logs iguais ou acima de determinado \
            nível, padrão: info"
    )
    leitor_args.add_argument(
        "-p", "--porta", type=int, default=8080,
        help="Porta que o servidor vai receber conexões, padrão: 8080"
    )
    leitor_args.add_argument(
        "--host", default="0.0.0.0",
        help="hostname do servidor, por onde os usuários vão se conectar,\
             padrão: 0.0.0.0"
    )

    args = leitor_args.parse_args()

    if args.help:
        leitor_args.print_help()
        exit(0)

    
    return args

if __name__ == "__main__":
    args = lerArgumentos()
    
    configurarLogging(nivel_logs_dict[args.nivel], args.json)
    logger = structlog.get_logger("main")
    logger.info("Inicializando programa")

    #utiliza o módulo waitress para servir o programa pois o servidor integrado
    #do flask é apenas para desenvolvimento
    waitress.serve(servidor, host=args.host, port=args.porta)