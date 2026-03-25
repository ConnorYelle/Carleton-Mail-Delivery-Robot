import logging
from rich.logging import RichHandler

class Logger:

    def __init__(self):
        pass

    def log_llm_query(self, message: str):
        logging.basicConfig(
            level="LLM QUERY", 
            format="%(message)",
            datefmt="[%X]",
            handlers=[RichHandler(rich_tracebacks=True)]
        )

        logger = logging.getLogger("rich")
        logger.info(message)

    def log_llm_response(self, message: str):
        logging.basicConfig(
            level="LLM RESPONSE", 
            format="%(message)",
            datefmt="[%X]",
            handlers=[RichHandler(rich_tracebacks=True)]
        )

        logger = logging.getLogger("rich")
        logger.info(message)

    