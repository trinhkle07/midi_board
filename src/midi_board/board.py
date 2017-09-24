"""A MIDI Board virtualization."""

from midi_board import midi_utility
from midi_board.controls.control import Control
from midi_board.custom_logging import LOGGER
from midi_board.virtual_midi import VirtualMIDI


class MidiBoard(VirtualMIDI):
    """A real MIDI board."""
    DEFAULT_MIDI_PORT = '{serial}:{serial} MIDI 1 24:0'

    def __init__(self, config):
        """Constructor."""
        VirtualMIDI.__init__(self, config)
        midi_port = self._config.get('midi_port', MidiBoard.DEFAULT_MIDI_PORT)
        port = midi_port.replace('{serial}', self._serial)
        midi_utility.initialize(port)
        midi_utility.register_callback(self)

    def finalize(self, *args, **kwargs):
        """Finalize and clean up."""
        VirtualMIDI.finalize(self, *args, **kwargs)
        midi_utility.close_ports()

    def __call__(self, event, data=None):

        message, deltatime = event
        channel, control_identifier, value = message

        LOGGER.debug('Received event on control %d | value=%d | deltatime=%d | channel=%s', control_identifier, value, deltatime, channel)
        control = self.get_control(control_identifier)
        if isinstance(control, Control):
            control.on_change(value)
