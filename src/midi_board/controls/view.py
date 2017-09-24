"""View Control GUI for a MIDI controller."""

from midi_board.controls.control import Control


class View(Control):
    """Parameter View"""

    def __init__(self, identifier, **kwargs):
        """Button Constructor."""
        Control.__init__(self, identifier, **kwargs)
        self._label = None
        self.data = None

    def on_timed_event(self):
        self._label.setText(self._label.objectName() + " = " + str(self.data))
