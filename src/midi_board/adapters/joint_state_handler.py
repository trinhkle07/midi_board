# Copyright (c) 2017 Houston Mechatronics Inc.
#
# Distribution of this file or its parts, via any medium is strictly
# prohibited. Permission to use must be explicitly granted by Houston
# Mechatronics Inc.
#
# Author: Trinh  Le <tle@houstonmecatronics.com>, July 2017

import midi_board.utility as utils
import rospy
from midi_board.adapters import ROSPublisher
from sensor_msgs.msg import JointState

from adapter import Adapter


class JointStateHandler(Adapter):
    DEFAULT_NAME = "MyJointStateHandler"

    def __init__(self, name=DEFAULT_NAME, topic=None, message_type=None, sub_topic=None, sub_message_type=None,
                 queue_size=10, **kwargs):
        Adapter.__init__(self, name, **kwargs)

        msg_type_class = utils.attempt_import(message_type, _myvars=vars())
        self._js_publisher = ROSPublisher(name=name, topic=topic, message_type=msg_type_class, queue_size=queue_size,
                                          **kwargs)

        if sub_topic is not None and sub_message_type is not None:
            sub_msg_type_class = utils.attempt_import(sub_message_type, _myvars=vars())
            self._js_subscriber = rospy.Subscriber(sub_topic, sub_msg_type_class,
                                                   self.callback) if sub_topic is not None else None
        self._js = None
        self._new_js = JointState()

    def finalize(self, *args, **kwargs):
        print("Closing ROS connection..")
        if not rospy.is_shutdown():
            print("Shutting down ROS node..")
            rospy.signal_shutdown("ROS node shut down.")

    def send_default_states(self, control):
        data = control.default_data
        js = JointState()
        js.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        js.position = []

        for name in js.name:
            js.position.append(data[name])

        js.header.stamp = rospy.get_rostime()
        js.header.frame_id = "/world"
        self._js_publisher.publish(js)

    def send_elbow_joint_state(self, control):
        self._write(control)

    def send_shoulder_lift_joint_state(self, control):
        self._write(control)

    def send_shoulder_pan_joint_state(self, control):
        self._write(control)

    def send_wrist_1_joint_state(self, control):
        self._write(control)

    def send_wrist_2_joint_state(self, control):
        self._write(control)

    def send_wrist_3_joint_state(self, control):
        self._write(control)

    def _write(self, control):  # slider control
        if self._js is None:
            # Log error
            return

        self._new_js.name = list(self._js.name)
        self._new_js.position = list(self._js.position)
        for idx, name in enumerate(self._new_js.name):
            if name == control.name:
                self._new_js.position[idx] = control.value
        self._new_js.header.stamp = rospy.get_rostime()
        self._new_js.header.frame_id = "/world"
        self._js_publisher.publish(self._new_js)

    def callback(self, data):
        self._js = data

    def get_param_value(self, control):  # view control
        if self._js is None:
            return
        names = list(self._js.name)
        positions = list(self._js.position)
        for idx, name in enumerate(names):
            if control.name == name:
                control.data = positions[idx]
