"""Lighthouse Adapter."""

import logging

import lighthouse_py
from deep_dive_msgs import aquanaut_msg_pb2
from midi_board.adapters.adapter import Adapter


class LighthouseAdapter(Adapter):
    """An Adapter for Lighthouse."""

    def __init__(self, name='LighthouseAdapter', log_level=logging.INFO, **kwargs):
        """Construct LighthouseAdapter."""
        Adapter.__init__(self, name, **kwargs)
        self._log_level = log_level
        logging.basicConfig(level=log_level, **kwargs)
        self._logger = logging.getLogger('LighthouseAdapter::{}'.format(self._name))

        # create instances of our proto message
        # or we could create a new instance each time in write() for some reason
        self._aquanaut_cmd = aquanaut_msg_pb2.AquanautCmd()
        self._actuator_cmd = self._aquanaut_cmd.actuator_commands.add()  # pylint: disable=no-member

        # lighthouse stuff
        self._handle = lighthouse_py.NodeHandle('deepdive')
        self._publisher = self._handle.advertise('actuator_control')

    def finalize(self, *args, **kwargs):
        # TODO: Should close Ethercat connection here. To be implemented
        pass

    def send_defaults(self, event):
        # TODO: Send default pos, vel, and effort values in config. To be implemented.
        pass

    def send_position(self, event):
        """Send a position command."""
        self.reset_message()
        self._actuator_cmd.name = event.name
        self._actuator_cmd.position = event.value
        self._actuator_cmd.mode = aquanaut_msg_pb2.POSITION
        self.write()

    def send_velocity(self, event):
        """Send a velocity command."""
        self.reset_message()
        self._actuator_cmd.name = event.name
        self._actuator_cmd.velocity = event.value
        self._actuator_cmd.mode = aquanaut_msg_pb2.VELOCITY

        self.write()

    def send_effort(self, event):
        """Send an effort command."""
        self.reset_message()
        self._actuator_cmd.name = event.name
        self._actuator_cmd.effort = event.value
        self._actuator_cmd.mode = aquanaut_msg_pb2.EFFORT

        self.write()

    def clear_faults(self, event):
        """Send a CLEAR_FAULTS message."""
        self.reset_message()
        self._actuator_cmd.name = event.name
        self._actuator_cmd.mode = aquanaut_msg_pb2.CLEAR_FAULTS
        self.write()

    def reset_message(self):
        """Resets the position, velocity, effort fields to 0."""
        # set all positions to 0 just to be sure
        self._actuator_cmd.position = 0
        self._actuator_cmd.velocity = 0
        self._actuator_cmd.effort = 0

    def write(self):
        """Send a command via Lighthouse."""
        # replace with lighthouse call(s)
        print(self._aquanaut_cmd, type(self._aquanaut_cmd))
        self._publisher.publish(self._aquanaut_cmd)
