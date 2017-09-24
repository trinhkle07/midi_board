"""An series of generic ROS adapters."""

import rospy
from midi_board.custom_logging import LOGGER
from rospy_message_converter.message_converter import convert_dictionary_to_ros_message


class ROSAdapter(object):
    """An abstract ROSAdapter."""

    DEFAULT_NAME = 'ROSAdapter'

    def __init__(self, name=DEFAULT_NAME, **kwargs):
        self.name = name
        if rospy.get_node_uri() is None:
            rospy.init_node('midi_board_ros_{}'.format(self.name))


class ROSPublisher(ROSAdapter):
    """A standard publisher adapter."""

    def __init__(self, name=ROSAdapter.DEFAULT_NAME, topic=None, message_type=None, data=None, queue_size=10, **kwargs): # pylint: disable=R0913
        ROSAdapter.__init__(self, name, **kwargs)
        self._topic = topic
        self._message_type = message_type
        self._message = message_type() if message_type is not None else None
        self._queue_size = queue_size
        self.initialize()
        self._message = self.update_msg(data) if data is not None else None

    def initialize(self):
        """Setup and configure the publisher."""
        if self._topic and self._message_type:
            LOGGER.info('<%s>[%s] -- Creating ROS.Publisher(%s, %s)', type(self).__name__, self.name, self._topic, self._message_type)
            self._publisher = rospy.Publisher(self._topic, self._message_type, queue_size=self._queue_size)
        else:
            LOGGER.warning('<%s>[%s] -- Must specify topic and message type! (%s, %s)',
                           type(self).__name__, self.name, self._topic, self._message_type)
            self._publisher = None

    def _publish(self, msg):
        LOGGER.debug('<%s>[%s] -- publish(%s)', type(self).__name__, self.name, msg)
        if isinstance(self._publisher, rospy.Publisher):
            if isinstance(msg, self._message_type):
                self._publisher.publish(msg)

    def publish(self, msg=None):
        """Publish a msg."""
        if not isinstance(msg, self._message_type):
            msg = self._message
        self._publish(msg)

    def update_msg(self, data, message=None, update_header=True):
        """Update a message."""
        msg = message or self._message
        #if data:
        #TODO: for data in data: update data # pylint: disable=fixme
        msg_type_str = self._message_type.__module__.replace('.msg._', '/')
        msg = convert_dictionary_to_ros_message(msg_type_str, data)
        if update_header:
            try:
                msg.header.stamp = rospy.get_rostime()
            except AttributeError:
                pass
        return msg

    def update_msg_send(self, data):
        """Update and send a message."""
        msg = self.update_msg(data)
        self._publish(msg)
