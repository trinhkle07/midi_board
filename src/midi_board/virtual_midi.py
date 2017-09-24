"""A virtualized MIDI controller."""

import midi_board.utility as utils
from midi_board.adapters import Adapter
from midi_board.controls.control import Control
from midi_board.custom_logging import LOGGER


class VirtualMIDI(object):
    """A virtual MIDI controller."""

    def __init__(self, config):
        """Create a virtual midi."""
        self._serial = None
        self._controls = {}
        self._control_map = {'name': {}, 'description': {}}
        self._adapters = {}
        self._user_config_description = ''
        self._config = None
        if isinstance(config, str):
            config = utils.load_from_file(config)
        self.load_config(**config)

    def __str__(self):
        serial = ':{}'.format(self._serial) if self._serial is not None else ''
        string = 'A {}{} with {} control(s) and {} adapter(s):'.format(type(self).__name__, serial, len(self._controls), len(self._adapters))
        return string

    def load_config(self, **config):
        """Load config data."""
        self._config = config
        self._setup_config(**config)

    def finalize(self, *args, **kwargs):
        """Cleanup the object and shutdown."""
        for adapter in self._adapters.values():
            adapter.finalize(*args, **kwargs)

    def add_control(self, control):
        """Add a control to this board."""
        LOGGER.debug('VirtualMIDI::add_control() -- Adding %s control to %s', control, self._serial)
        if control.identifier in self._controls:
            LOGGER.warning('VirtualMIDI::add_control() -- Overwriting control %s', control.identifier)
        self._controls[control.identifier] = control
        self._control_map['name'][control.name] = control
        self._control_map['description'][control.description] = control

    def add_adapter(self, adapter):
        """Add a control to this board."""
        LOGGER.debug('VirtualMIDI::add_adapter() -- Adding %s adapter to %s', adapter, self._serial)
        if adapter.name in self._adapters:
            LOGGER.warning('VirtualMIDI::add_adapter() -- Overwriting adapter %s', adapter.name)
        self._adapters[adapter.name] = adapter

    @staticmethod
    def connect(control, signal, adapter, callback):
        """Connect a control::signal to an adapter::callback for notification."""
        LOGGER.info('VirtualMIDI::connect %s::%s <--> %s::%s', control, signal, adapter, callback.__name__)
        control.add_observer(adapter, signal, callback)
        adapter.set_callback(control, signal, callback)

    def get_controls(self, control_type=None):
        """Retrieve list of controls by control type."""
        if control_type is not None:
            controls = []
            for control in self._controls.values():
                if isinstance(control, control_type):
                    controls.append(control)
            return controls
        else:
            return self._controls

    def get_control(self, identifier=None, name=None, description=None):
        """Retrieve a control by ID, name, or description."""
        if identifier is not None:
            if identifier in self._controls:
                return self._controls[identifier]
        if name is not None:
            if name in self._control_map['name']:
                return self._control_map['name'][name]
        if description is not None:
            if name in self._control_map['description']:
                return self._control_map['description'][description]

    def get_adapter(self, name):
        """Retrieve an adapter by name."""
        if isinstance(name, Adapter):
            if name.name in self._adapters:
                if self._adapters[name.name] == name:
                    return name
        if name in self._adapters:
            return self._adapters[name]

    def _setup_board_config(self, serial=None, controls=None, **kwargs):# pylint: disable=unused-argument
        self._serial = serial
        LOGGER.info('Configuring VirtualMIDI: %s', serial)
        for control_type in controls.get('types', []):
            control_params = control_type.copy()
            control_type_class = utils.attempt_import(control_params['type'], _myvars=vars())
            del control_params['type']
            for control in controls.get(control_type['name']):
                this_control_params = control_params.copy()
                this_control_params.update(control)
                ctrl = Control.create_control(control_type_class, **this_control_params)
                self.add_control(ctrl)

    def _setup_config(self, config=None, name=None, adapters=None, controls=None, **kwargs):# pylint: disable=unused-argument
        self._user_config_description = name
        if isinstance(config, str):
            config = utils.load_from_file(config)
        if not isinstance(config, dict):
            raise TypeError('MIDI board config file not specified.')
        self._setup_board_config(**config)
        for adapter_type in adapters:
            adapter_params = adapter_type.copy()
            adapter_type_class = utils.attempt_import(adapter_params['type'], _myvars=vars())
            del adapter_params['type']
            adapter = Adapter.create_adapter(adapter_type_class, **adapter_params)
            self.add_adapter(adapter)

        for control_type in controls:
            for control in controls.get(control_type):
                try:
                    self._configure_control(control)
                except KeyError as key_err:
                    LOGGER.warning('VirtualMIDI::_setup_user_config() -- Malformed control; requires key: %s', key_err)
                except AttributeError as att_err:
                    LOGGER.warning('VirtualMIDI::_setup_user_config() -- Unknown callback: %s', att_err)

    def _configure_control(self, control_data):
        control = self._controls.get(control_data['identifier'], None)
        if not control:
            control_type = control_data['type']
            control_type_class = utils.attempt_import(control_type, _myvars=vars())
            control = Control.create_control(control_type_class, **control_data)
            self.add_control(control)
            LOGGER.warning('VirtualMIDI::_configure_control() -- Control.identifier %s not found. '
                           'This control is created virtually..', control_data['identifier'])

        for key in control.properties:
            try:
                old_key = getattr(control, key)
                setattr(control, key, control_data[key])
                if old_key in self._control_map[key]:
                    del self._control_map[key][old_key]
                self._control_map[key][control_data[key]] = control
            except KeyError:
                pass
        for signal_name in control_data['signals']:
            signal = control_data['signals'][signal_name]
            self._configure_control_signal(control, signal_name, signal)
        return True

    def _configure_control_signal(self, control, signal_name, signal):
        if isinstance(signal, list):
            for a_signal in signal:
                self._configure_control_signal(control, signal_name, a_signal)
            return True
        if not isinstance(signal, str):
            LOGGER.warning('VirtualMIDI::_configure_control_signal() -- Bad yaml data: signal data must be string or list of strings.')
            return False
        adapter_name, callback_name = signal.split('.')
        if 'this' in adapter_name:
            adapter = control
        else:
            adapter = self._adapters.get(adapter_name)
        if not adapter:
            LOGGER.warning('VirtualMIDI::_configure_control_signal() -- Unknown adapter: %s', adapter_name)
            return False
        callback = getattr(adapter, callback_name)
        signal = getattr(type(control), signal_name)
        if not signal:
            LOGGER.warning('VirtualMIDI::_configure_control_signal() -- Unknown signal: %s', signal_name)
            return False
        self.connect(control, signal, adapter, callback)
        return True
