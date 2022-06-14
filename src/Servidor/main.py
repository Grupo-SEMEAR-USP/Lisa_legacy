#!/usr/bin/env python3
import structlog
import logging
import waitress

from servidor import servidor


if __name__ == "__main__":
    json = False

    renderer = structlog.processors.JSONRenderer() if json else structlog.dev.ConsoleRenderer()

    structlog.configure(
        processors=[
            structlog.stdlib.ProcessorFormatter.wrap_for_formatter,
        ],
        logger_factory=structlog.stdlib.LoggerFactory(),
    )

    def dropunderscores(logger, log_method, event_dict):
        underscores = [k for k in event_dict.keys() if k[0]=="_"]
        for k in underscores:
            event_dict.pop(k)
        return event_dict

    formatter = structlog.stdlib.ProcessorFormatter(
        processors=[
            structlog.stdlib.add_logger_name,
            structlog.stdlib.add_log_level,
            dropunderscores,
            structlog.processors.format_exc_info,
            structlog.processors.StackInfoRenderer(),
            structlog.processors.TimeStamper("%X", utc=False),
            renderer
        ],
    )

    handler = logging.StreamHandler()
    handler.setFormatter(formatter)
    root_logger = logging.getLogger()
    root_logger.addHandler(handler)
    root_logger.setLevel(logging.DEBUG)


    waitress.serve(servidor, host="0.0.0.0", port=8080)