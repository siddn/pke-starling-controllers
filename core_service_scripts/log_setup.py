import logging

def setup_logging(name=__name__, log_file=None, log_level='DEBUG'):
    logger = logging.getLogger(__name__)
    logger.setLevel(getattr(logging, log_level.upper(), logging.DEBUG))
    if logger.hasHandlers(): logger.handlers.clear()
    if log_file:
        handler = logging.FileHandler(log_file)
    else:
        handler = logging.StreamHandler()
    handler.setLevel(getattr(logging, log_level.upper(), logging.DEBUG))
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)

    return logger