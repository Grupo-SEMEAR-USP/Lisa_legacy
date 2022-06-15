#!/usr/bin/env python3
import structlog
import logging
import waitress
import argparse
import sys

from servidor import servidor


def tirarUnderscores(logger, log_method, event_dict):
    
    underscores = [k for k in event_dict.keys() if k[0]=="_"]
    for k in underscores:
        event_dict.pop(k)
    return event_dict


def configurarLogging(json=False, debug=False):
    
    if json:
        renderizador = structlog.processors.JSONRenderer()
    else:
        renderizador = structlog.dev.ConsoleRenderer()
    
    nivel_logs = logging.DEBUG if debug else logging.WARN

    structlog.configure(
        processors=[
            structlog.stdlib.ProcessorFormatter.wrap_for_formatter,
        ],
        logger_factory=structlog.stdlib.LoggerFactory(),
    )


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

    executor_logs = logging.StreamHandler()
    executor_logs.setFormatter(formatador)

    logger_base = logging.getLogger()
    logger_base.addHandler(executor_logs)
    logger_base.setLevel(nivel_logs)


def lerArgumentos():
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
        help="Realiza logs por JSON ao invés da interface de terminal, padrão: Falso"
    )
    leitor_args.add_argument(
        "-d", "--debug", action="store_true",
        help="Inicia o programa em modo de debug, padrão: Falso"
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
    
    configurarLogging(args.json, args.debug)

    waitress.serve(servidor, host=args.host, port=args.porta)