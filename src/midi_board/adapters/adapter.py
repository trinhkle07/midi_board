"""Adapter Tools."""

import logging

from midi_board.custom_logging import LOGGER
from midi_board.custom_logging import setup_logging
from midi_board.utility import Observer
from midi_board.utility import attempt_import


class Adapter(Observer):
    """An adapter to forward control event data to."""

    def __init__(self, name, description='', **kwargs):  # pylint: disable=unused-argument
        """Construct Adapter."""
        Observer.__init__(self)
        self._name = name
        self._description = description
        self._callback_map = {}

    @property
    def name(self):
        """Get the name of this adapter."""
        return self._name

    def __str__(self):
        """Convert to human readbale string."""
        string = '<{}>[{}]'.format(type(self).__name__, self.name)
        if self._description:
            string = '{}: {}'.format(string, self._description)
        return string

    def finalize(self, *args, **kwargs):
        """Clean up and shutdown."""
        pass

    def notify(self, signal, control, *args, **kwargs): # pylint: disable=W0221
        """Notify this object of an event."""
        LOGGER.info('Adapter::notify(%s, %s, %s, %s)', signal, control, args, kwargs)
        try:
            for callback in self._callback_map[control][signal]:
                callback(control)
        except KeyError as key_err:
            LOGGER.warning('Adapter::notify() failed to find key %s in %s', key_err, self._callback_map)

    def set_callback(self, control, signal, callback, index=None):
        """Add a callback."""
        if control not in self._callback_map:
            self._callback_map[control] = {}
        if signal not in self._callback_map[control]:
            self._callback_map[control][signal] = []
        index = index if isinstance(index, int) else len(self._callback_map[control][signal])
        self._callback_map[control][signal].insert(index, callback)

    @staticmethod
    def create_adapter(adapter_type, **kwargs):
        """Create an adapter from kwargs."""
        adapter_type = attempt_import(adapter_type)
        adapter = adapter_type(**kwargs)
        return adapter


class DebugAdapter(Adapter):
    """An Adapter for debugging."""

    DEFAULT_NAME = 'DefaultDebugAdapter'
    DEFAULT_LOG_LEVEL = logging.INFO

    def __init__(self, name=DEFAULT_NAME, log_level=DEFAULT_LOG_LEVEL, **kwargs):
        """Construct DebugAdapter."""
        Adapter.__init__(self, name, **kwargs)
        self._log_level = log_level
        self._logger = logging.getLogger('DebugAdapter::{}'.format(self._name))
        setup_logging(self._logger)

    def on_event(self, *args, **kwargs):
        """Call on a new event."""
        event = args, kwargs
        self.log_event(event)

    def log_event(self, event):
        """Log an event."""
        self._logger.info(event)

    def warning(self, event):
        """Log an event using a warning."""
        self._logger.warning(event)
