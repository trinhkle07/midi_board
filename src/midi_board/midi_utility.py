# Copyright (c) 2017 Houston Mechatronics Inc.
#
# Distribution of this file or its parts, via any medium is strictly
# prohibited. Permission to use must be explicitly granted by Houston
# Mechatronics Inc.
#
# Author: Trinh  Le <tle@houstonmecatronics.com>, July 2017

import rtmidi
from midi_board.custom_logging import LOGGER

MIDI_IN = rtmidi.MidiIn()
MIDI_OUT = rtmidi.MidiOut()
MIDI_PORT_NAME = None


def initialize(port):
    """Connect to the midi board."""
    global MIDI_IN, MIDI_OUT, MIDI_PORT_NAME
    MIDI_PORT_NAME = port
    if not MIDI_IN.is_port_open():
        try:
            ports = MIDI_IN.get_ports()
            if port in ports:
                port_number = ports.index(port)
                MIDI_IN.open_port(port_number)
                MIDI_OUT.open_port(port_number)
                LOGGER.info('Successfully connected to board')
            else:
                LOGGER.warning('Failed to connect to board')
                return False
        except rtmidi.RtMidiError as midi_error:
            LOGGER.warning('Unable to connect to MIDI board\n\t%s', midi_error)
            return False
    return True


def register_callback(callback):
    MIDI_IN.set_callback(callback)


def send_msg(msg):
    MIDI_OUT.send_message(msg)


def close_ports():
    print("Closing ports..")
    global MIDI_IN, MIDI_OUT
    MIDI_IN.close_port()
    MIDI_OUT.close_port()


def is_board_plugged():
    global MIDI_IN, MIDI_PORT_NAME
    if MIDI_PORT_NAME in MIDI_IN.get_ports():
        return True
    return False
