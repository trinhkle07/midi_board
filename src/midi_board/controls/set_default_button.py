from __future__ import division

from midi_board.adapters.adapter import Adapter
from midi_board.controls.control import Control


class SetDefaultButton(Control, Adapter):

    def __init__(self, identifier, **kwargs):
        """Button Constructor."""
        Control.__init__(self, identifier, **kwargs)
        self.default_data = None
        self._sliders = []
        self._encoder_buttons = []
        self._button = None

    def on_default_button_clicked_cb(self, default_values):
        self.on_clicked(default_values)

    def on_clicked(self, default_values):
        self.default_data = default_values

    def update_qt_controls_default(self, control):

        for slider in self._sliders:
            if slider.default_value == (slider.RAW_MAX_VALUE - slider.RAW_MIN_VALUE) / 2.0:
                # for the case where min/max are set but default_value is set in robot config
                # default_value still 63.5. So, we need to call get_center() again to correct center value
                slider.default_value = slider.get_center()
            slider._val_set.setText(str(slider.default_value))
            slider._slider.setValue(slider.default_raw_value)
            slider.raw_value = slider.default_raw_value
            slider.move_slider(slider)

        for encoder_button in self._encoder_buttons:
            if encoder_button.default_value == (encoder_button.RAW_MAX_VALUE - encoder_button.RAW_MIN_VALUE) / 2.0:
                encoder_button.default_value = encoder_button.get_center()
            encoder_button._val_set.setText(str(encoder_button.default_value))
            encoder_button._dial.setValue(encoder_button.default_raw_value)
            encoder_button.raw_value = encoder_button.default_raw_value
            encoder_button.move_dial(encoder_button)
