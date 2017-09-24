"""Controls for a MIDI controller."""

from midi_board.utility import attempt_import
from midi_board.utility import ObservableEncapsulator


class Control(ObservableEncapsulator):
    """A generic control."""

    def __init__(self, identifier, name='', description='', **kwargs): # pylint: disable=unused-argument
        """Constructor."""
        ObservableEncapsulator.__init__(self)
        self._identifier = identifier
        self._name = name
        self._description = description
        self.properties = ('name', 'description')

    @property
    def name(self):
        """Get the name of this control."""
        return self._name

    @name.setter
    def name(self, val):
        """Set the name of the control."""
        self._name = val

    @property
    def description(self):
        """Get the description of this control."""
        return self._name

    @description.setter
    def description(self, val):
        """Set the description of the control."""
        self._description = val

    @property
    def identifier(self):
        """Get the name of this adapter."""
        return self._identifier

    def __str__(self):
        """Convert to human readbale string."""
        string = '<{}>[{}:{}]'.format(type(self).__name__, self.name, self.identifier)
        if self._description:
            string = '{}: {}'.format(string, self._description)
        return string

    @staticmethod
    def create_control(control_type, **kwargs):
        """Create a control from a dict of kwargs."""
        control_type = attempt_import(control_type)
        return control_type(**kwargs)

    def on_change(self, value):
        """Abstract method must be implemented to change the state of the control."""
        raise NotImplementedError('{} must implement on_change function. Got value: {}'.format(type(self).__name__, value))


class ControlWritable(Control):
    """A generic control that can be written to."""

    def __init__(self, identifier, write_protected=False, **kwargs):
        """Constructor."""
        Control.__init__(self, identifier, **kwargs)
        self._write_protected = write_protected

    def write(self, event):
        """Write the data to the object."""
        pass

    def on_change(self, value):
        """Abstract method must be implemented to change the state of the control."""
        raise NotImplementedError('CW{} must implement on_change function. Got value: {}'.format(type(self).__name__, value))
