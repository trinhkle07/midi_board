"""Common midi_board tools."""

import abc
import importlib
import rospkg  # pylint: disable=E0401
import types

import yaml  # pylint: disable=E0401
from midi_board.custom_logging import LOGGER

# python 2/3 forward compatibility
if not hasattr(types, 'TypeType'):
    types.TypeType = type
if not hasattr(types, 'UnboundMethodType'):
    types.UnboundMethodType = types.FunctionType


def load_from_file(filename):
    """Load yaml data from a file."""
    data = None
    filename = get_uri(filename)
    with open(filename) as file:
        data = yaml.load(file.read())
    return data

def attempt_import(string, **kwargs):
    """Attempt an import from string data.

    Raises:
        ImportError

    """
    if isinstance(string, types.TypeType):
        return string
    elif isinstance(string, str):
        try:
            index = string.rindex('.')
            pkg, typ = string[:index], string[index + 1:]
            if '_myvars' in kwargs and typ in kwargs['_myvars']:
                return kwargs['_myvars'][typ]
            return getattr(importlib.import_module(pkg), typ)
        except Exception as exception:  # pylint: disable=broad-except
            raise ImportError('Failed to import {}:{}'.format(string, exception))
    raise ImportError('Failed to import {}:{}'.format(type(string), string))


def get_uri(path):
    """Get the path to the file.

    Raises:
        rospkg.ResourceNotFound

    """
    mod_url = path
    if path.find('package://') == 0:
        mod_url = path[10:]
        pos = mod_url.find('/')
        if pos == -1:
            raise rospkg.ResourceNotFound("Could not parse package:// format into raw URI path format for " + path)
        package = mod_url[0:pos]
        mod_url = mod_url[pos:]
        package_path = rospkg.RosPack().get_path(package)
        mod_url = package_path + mod_url
    elif path.find('file://') == 0:
        mod_url = path[7:]
    return mod_url


class Observer(object):
    """An Observer."""

    __metaclass__ = abc.ABCMeta

    def __init__(self):
        """Constructor."""
        pass

    @abc.abstractmethod
    def notify(self, *args, **kwargs):
        """Update the object with new information."""
        LOGGER.info('Observer::notify(%s, %s)', args, kwargs)

    @abc.abstractmethod
    def finalize(self, *args, **kwargs):
        """Cleanup the object and shutdown."""
        LOGGER.info('Observer::finalize(%s, %s)', args, kwargs)


class Observable(object):
    """An observable object."""

    def __init__(self):
        """Constructor."""
        self._observers = []

    def add_observer(self, observer):
        """Add and observer."""
        if observer not in self._observers:
            self._observers.append(observer)
            return True
        return False

    def remove_observer(self, observer):
        """Remove an observer."""
        if observer in self._observers:
            self._observers.remove(observer)
            return True
        return False

    def remove_observers(self):
        """Remove all observers."""
        self._observers = []
        return True

    def notify_observers(self, *args, **kwargs):
        """Notify all observers."""
        for observer in self._observers:
            observer.notify(*args, **kwargs)


class ObservableEncapsulator(Observable):
    """An object which notifies observer."""

    def __init__(self):  # , name):
        """Construct FeedbackObject."""
        Observable.__init__(self)
        self._callback_map = {}
        self._function_remap = {}
        # self._name = name

    def add_observer(self, observer, signal=None, index=None): # pylint: disable=W0221
        """Add an observer to the FeedbackObject.

        Note:
            If included, args and kwargs (updated) will be passed to the function

        Args:
            observer (Observer): observer to notify.
            signal (function): the signal feedback type to bind the observer to.
            *args: Variable length argument list.
            **kwargs: Arbitrary keyword arguments.

        """
        if not signal:
            return Observable.add_observer(self, observer)
        if repr(signal).startswith('<bound'):
            signal = getattr(type(self), signal.__name__)
        if signal in self._function_remap:
            signal = self._function_remap[signal]
        if not isinstance(signal, types.UnboundMethodType):
            raise TypeError("Invalid signal type {}; expected UnboundMethodType".format(type(signal).__name__))
        if signal not in self._callback_map:
            self._callback_map[signal] = []
            self._wrap_callback(signal)
        if observer in self._callback_map[signal]:
            return False
        if not isinstance(observer, Observer):
            raise TypeError("Invalid observer type {}; expected instace of Observer".format(type(signal).__name__))
        index = index if isinstance(index, int) else len(self._callback_map[signal])
        self._callback_map[signal].insert(index, observer)
        return True

    def remove_observer(self, observer, signal=None):  # pylint: disable=W0221
        """Remove an observer."""
        if signal in self._callback_map:
            self._callback_map[signal].remove(observer)
            return True
        success = Observable.remove_observer(self, observer)
        for a_signal in self._callback_map:
            if observer in self._callback_map[a_signal]:
                self._callback_map[a_signal].remove(observer)
                success = True
        return success

    def remove_observers(self, signal=None):  # pylint: disable=W0221
        """Remove all observers."""
        if signal in self._callback_map:
            self._callback_map[signal] = []
        if signal is None:
            self._callback_map = {}
            return Observable.remove_observers(self)
        return True

    def notify_observers(self, signal, *args, **kwargs):  # pylint: disable=W0221
        """Notify all observers."""
        if signal not in self._callback_map:
            Observable.notify_observers(self, *args, **kwargs)
        else:
            for observer in self._callback_map[signal]:
                observer.notify(signal, self, *args, **kwargs)

    def _wrap_callback(self, callback):
        my_callback = getattr(self, callback.__name__)
        my_callback_key = getattr(type(self), callback.__name__)

        # wrap this function so it will notify on after call
        def __encapsulated_callback(*args, **kwargs):
            my_callback(*args, **kwargs)
            self.notify_observers(my_callback_key, *args, **kwargs)

        # Keep track of the function remap
        self._function_remap[__encapsulated_callback] = my_callback_key
        setattr(self, callback.__name__, __encapsulated_callback)
