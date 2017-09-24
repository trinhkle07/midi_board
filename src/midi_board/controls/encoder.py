"""Encoder Controls for a MIDI controller."""

from __future__ import division

from midi_board.controls import Control


class Encoder(Control):
    """A Encoder control."""

    RAW_MIN_VALUE = 0
    RAW_MAX_VALUE = 127

    DEFAULT_NAME = 'encoder'
    DEFAULT_DESCRIPTION = 'An encoder.'

    def __init__(self, identifier, min_value=RAW_MIN_VALUE, max_value=RAW_MAX_VALUE, default_value=None, **kwargs):
        """Construct Encoder."""
        Control.__init__(self, identifier, **kwargs)
        self._raw_value = None
        self._min_value = min_value
        self._max_value = max_value
        self._default_value = default_value or self.get_center()
        self._default_raw_value = self.update_default_raw()
        self._scaled_value = self._default_value
        self.properties += ('max_value', 'min_value', 'default_value')

    @property
    def default_raw_value(self):
        """Get the default raw value of the slider."""
        return self._default_raw_value

    @property
    def raw_value(self):
        """Get the raw value of the slider."""
        return self._raw_value

    @raw_value.setter
    def raw_value(self, val):
        """Set the raw value of the slider."""
        self._raw_value = val
        self.update_scaled_val()

    @property
    def value(self):
        """Get the scaled value of the slider."""
        return self._scaled_value

    @value.setter
    def value(self, val):
        """Set the scaled value of the slider."""
        self._scaled_value = val
        self.update_raw_val()

    @property
    def max_value(self):
        """Get the maximum value of the slider."""
        return self._max_value

    @max_value.setter
    def max_value(self, val):
        """Set the maximum value of the slider."""
        self._max_value = val
        self.update_raw_val()
        self._default_raw_value = self.update_default_raw()

    @property
    def min_value(self):
        """Get the minimum value of the slider."""
        return self._min_value

    @min_value.setter
    def min_value(self, val):
        """Set the minimum value of the slider."""
        self._min_value = val
        self.update_raw_val()
        self._default_raw_value = self.update_default_raw()

    @property
    def default_value(self):
        """Get the default value of the slider."""
        return self._default_value

    @default_value.setter
    def default_value(self, val):
        """Set the default value of the slider."""
        self._default_value = val
        self._default_raw_value = self.update_default_raw()

    def on_change(self, value):
        """Signal the value has changed."""
        self._raw_value = value
        self.update_scaled_val()

    def get_center(self):
        """Compute the center value of the slider."""
        center = (self._max_value - self._min_value) / 2.0
        return center

    def update_default_raw(self):
        default_raw_value = None
        if self._default_value is not None:
            if self._min_value is None and self._max_value is None:
                default_raw_value = (self._default_value - Encoder.RAW_MIN_VALUE) / \
                                  (Encoder.RAW_MAX_VALUE - Encoder.RAW_MIN_VALUE) * Encoder.RAW_MAX_VALUE
            elif self._min_value is not None and self._max_value is not None and self._min_value < self._max_value:
                default_raw_value = (self._default_value - self._min_value) / \
                                  (self._max_value - self._min_value) * Encoder.RAW_MAX_VALUE
            else:
                default_raw_value = None
        return default_raw_value

    def update_scaled_val(self):
        """Update value of the slider."""
        if self._raw_value is not None:
            if self._min_value is None and self._max_value is None:
                self._scaled_value = self._raw_value / Encoder.RAW_MAX_VALUE * (Encoder.RAW_MAX_VALUE - Encoder.RAW_MIN_VALUE) \
                                     + Encoder.RAW_MIN_VALUE
            elif self._min_value is not None and self._max_value is not None and self._min_value < self._max_value:
                self._scaled_value = self._raw_value / Encoder.RAW_MAX_VALUE * (self._max_value - self._min_value) \
                                     + self._min_value
            else:
                self._scaled_value = self._default_value

    def update_raw_val(self):
        if self._min_value is None and self._max_value is None:
            self._raw_value = (self._scaled_value - Encoder.RAW_MIN_VALUE) / \
                              (Encoder.RAW_MAX_VALUE - Encoder.RAW_MIN_VALUE) * Encoder.RAW_MAX_VALUE
        elif self._min_value is not None and self._max_value is not None and self._min_value < self._max_value:
            self._raw_value = (self._scaled_value - self._min_value) / \
                              (self._max_value - self._min_value) * Encoder.RAW_MAX_VALUE
        else:
            self._raw_value = None
