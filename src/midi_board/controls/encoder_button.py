"""Encoder Controls for a MIDI controller."""

from midi_board.adapters.adapter import Adapter
from midi_board.controls.button import Button
from midi_board.controls.encoder import Encoder
from midi_board import midi_utility


class EncoderButton(Encoder, Button, Adapter):
    """An Encoder with Button control."""

    DEFAULT_NAME = 'encoder'
    DEFAULT_DESCRIPTION = 'An encoder with a button.'

    def __init__(self, identifier, name=DEFAULT_NAME, description=DEFAULT_DESCRIPTION, **kwargs):
        """Construct EncoderButton."""
        Button.__init__(self, identifier, name=name, description=description, **kwargs)
        Encoder.__init__(self, identifier, name=name, description=description, **kwargs)
        Adapter.__init__(self, name=name, **kwargs)

        self._dial = None
        self._min = None
        self._max = None
        self._val_set = None
        self._control_param_name = None
        self._pop_up_dialog = None
        self._validator = None

    def update_qt_dial(self, control):
        self._dial.setValue(self._raw_value)
        self._val_set.setText(str(self.value))
        self._min.setText(str(self.min_value))
        self._max.setText(str(self.max_value))
        if self._dial.signalsBlocked():
            self._dial.blockSignals(False)

    def move_dial(self, control):
        if Encoder.RAW_MIN_VALUE <= self.raw_value <= Encoder.RAW_MAX_VALUE:
            message = [0xB0, self.identifier, self.raw_value]  # CONTROL_CHANGE = 0xB0. To remove rtmidi import.
            midi_utility.send_msg(message)

    def on_dial_changed_value_cb(self, val):
        self.on_dial_changed_value(val)

    def on_dial_changed_value(self, value):
        self.raw_value = value
        self._val_set.setText(str(self.value))

    def on_val_set_return_pressed_cb(self):
        self.on_val_set_return_pressed()

    def on_val_set_return_pressed(self):
        self.value = float(self._val_set.text())
        self._slider.setValue(self.raw_value)

    def on_range_edited_cb(self):
        self.on_range_edited()

    def on_range_edited(self):
        self._pop_up_dialog.curr_min = self.min_value
        self._pop_up_dialog.curr_max = self.max_value

        dialog_code = self._pop_up_dialog.exec_()
        if dialog_code == 1:  # Accepted

            if self._pop_up_dialog.new_min is None or self._pop_up_dialog.new_max is None \
                    or self._pop_up_dialog.new_min >= self._pop_up_dialog.new_max:
                self._pop_up_dialog.popup_err_msg.setText("Invalid range!")
                self._pop_up_dialog.popup_err_msg.setStyleSheet("color : red")
                self.on_range_edited()
                return

            self._pop_up_dialog.popup_err_msg.setText("")
            self.min_value = self._pop_up_dialog.new_min
            self.max_value = self._pop_up_dialog.new_max

            self._validator.setRange(self.min_value, self.max_value, 20)
            self._val_set.setValidator(self._validator)
            self._pop_up_dialog.close()
        if dialog_code == 0:  # Rejected
            self._pop_up_dialog.close()

        self._dial.blockSignals(True)
