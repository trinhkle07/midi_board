"""MIDI Logging functions."""

import logging

DEFAULT_LOG_LEVEL = logging.INFO
LOGGER_NAME = 'midi_board'

def get_logger(logger_name=LOGGER_NAME):
    """Get the logger for this module."""
    logger = logging.getLogger(logger_name)
    return logger

def setup_logging(logger=None, log_level=DEFAULT_LOG_LEVEL):
    """Initialize the logger for this module."""
    logger = logger or get_logger()
    logger.setLevel(log_level)


    # create console handler and set level to debug
    log_handler = logging.StreamHandler()

    # create formatter
    log_formatter = logging.Formatter('[%(levelname)s] [%(created)f]: %(message)s')

    # add formatter to ch
    log_handler.setFormatter(log_formatter)

    # add ch to logger
    logger.addHandler(log_handler)
    return logger

LOGGER = setup_logging()
