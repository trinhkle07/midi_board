"""Button Controls for a MIDI controller."""

from enum import Enum

from midi_board import midi_utility
from midi_board.adapters.adapter import Adapter
from midi_board.controls.control import Control
from midi_board.custom_logging import LOGGER


class Button(Control, Adapter):
    """A push button."""

    class State(Enum):
        """The state of a button."""
        OFF = 0
        ON = 127 # pylint: disable=invalid-name
        UNKNOWN = -1

    def __init__(self, identifier, default_state=State.UNKNOWN,
                 state_on=State.ON.value,
                 state_off=State.OFF.value, **kwargs):
        """Button Constructor."""
        Control.__init__(self, identifier, **kwargs)
        Adapter.__init__(self, **kwargs)
        self._state = default_state
        self._state_off = state_off
        self._state_on = state_on
        self.properties += ('state',)

    @property
    def state(self):
        """Get the button state."""
        return self._state

    @state.setter
    def state(self, val):
        """Set the button state."""
        if isinstance(self._state, Button.State):
            self._state = val

    def pressed(self):
        """Signal when button is pressed."""
        self.state = Button.State.ON

    def released(self):
        """Signal when button is released."""
        self.state = Button.State.OFF

    def is_pressed(self):
        """Determine if button is pressed."""
        return self.state == Button.State.ON

    def on_change(self, value):
        """Call to change the state of the button."""
        if value == self._state_on:
            self.pressed()
        elif value == self._state_off:
            self.released()
        else:
            LOGGER.warning('<%s>[%s] got UNKNOWN State value: %s', type(self).__name__, self.name, value)
            self.state = Button.State.UNKNOWN


class ButtonToggle(Button):
    """A push button with saved state."""

    class Switch(Enum):
        """The type of switching (state)."""
        PRESS = 3
        RELEASE = 4

    def __init__(self, identifier, switch=Switch.RELEASE, **kwargs):
        """Button Constructor."""
        Button.__init__(self, identifier, **kwargs)
        self._switch = switch
        self._active = Button.State.UNKNOWN

        self._button_toggle = None

    def toggle(self):
        """Toggle the button state."""
        if self._active == Button.State.ON:
            self._active = Button.State.OFF
        else:
            self._active = Button.State.ON

    def on_change(self, value):
        if value == self._state_on:
            self._active = Button.State.ON
        else:
            self._active = Button.State.OFF

    def update_qt_button(self, control):
        if self._active == Button.State.ON:
            self._button_toggle.setChecked(True)
        else:
            self._button_toggle.setChecked(False)

    def on_clicked_cb(self):
        self.on_clicked()

    def on_clicked(self):
        if self._button_toggle.isChecked():
            self._active = Button.State.ON
        else:
            self._active = Button.State.OFF

    def toggle_button(self, control):
        if self._active == Button.State.ON:
            message = [0xB0, self.identifier, 127]  # CONTROL_CHANGE = 0xB0. To remove rtmidi import
        else:
            message = [0xB0, self.identifier, 0]
        midi_utility.send_msg(message)