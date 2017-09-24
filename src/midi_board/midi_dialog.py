# Copyright (c) 2017 Houston Mechatronics Inc.
#
# Distribution of this file or its parts, via any medium is strictly
# prohibited. Permission to use must be explicitly granted by Houston
# Mechatronics Inc.
#
# Author: Trinh  Le <tle@houstonmecatronics.com>, July 2017

from PyQt5 import QtWidgets

from PyQt5.QtCore import Qt
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from midi_board.controls import ButtonToggle
from midi_board.controls import EncoderButton
from midi_board.controls import SetDefaultButton
from midi_board.controls import Slider
from midi_board.controls import View
from midi_board.popup_edit_dialog import PopupEditDialog

import midi_board_ui
import midi_utility


class MidiDialog(QDialog, midi_board_ui.Ui_Dialog_BCF2000):

    SLIDER_RAW_ID_TO_ID = {81: 0,
                           82: 1,
                           83: 2,
                           84: 3,
                           85: 4,
                           86: 5,
                           87: 6,
                           88: 7
                           }

    BUTTON_TOGGLE_RAW_ID_TO_ID = {65: 0,
                                  66: 1,
                                  67: 2,
                                  68: 3,
                                  69: 4,
                                  70: 5,
                                  71: 6,
                                  72: 7,
                                  73: 8,
                                  74: 9,
                                  75: 10,
                                  76: 11,
                                  77: 12,
                                  78: 13,
                                  79: 14,
                                  80: 15,
                                  }
    DIAL_RAW_ID_TO_ID = {1: 0, 2: 1, 3: 2, 4: 3, 5: 4, 6: 5, 7: 6, 8: 7,
                         9: 8, 10: 9, 11: 10, 12: 11, 13: 12, 14: 13, 15: 14, 16: 15,
                         17: 16, 18: 17, 19: 18, 20: 19, 21: 20, 22: 21, 23: 22, 24: 23,
                         25: 24, 26: 25, 27: 26, 28: 27, 29: 28, 30: 29, 31: 30, 32: 31
                         }

    DEFAULT_NAME = "MyMidiDialog"

    def __init__(self, midi):
        QDialog.__init__(self)
        self.setupUi(self)
        self._configure_qt_controls(midi)

    def _configure_qt_controls(self, midi):

        sliders = midi.get_controls(control_type=Slider)
        buttons = midi.get_controls(control_type=ButtonToggle)
        encoder_buttons = midi.get_controls(control_type=EncoderButton)
        views = midi.get_controls(control_type=View)
        set_defaults_button = midi.get_controls(control_type=SetDefaultButton)[0]
        default_values = {}

        for encoder_button in encoder_buttons:
            if encoder_button.identifier in MidiDialog.DIAL_RAW_ID_TO_ID.keys():
                id_ = MidiDialog.DIAL_RAW_ID_TO_ID.get(encoder_button.identifier)
                if id_ is not None:
                    encoder_button._dial = self.findChild(QDial, "dial_" + str(id_))
                    encoder_button._control_param_name = self.findChild(QLabel, "dial_control_param_name_" + str(id_))
                    encoder_button._min = self.findChild(QLineEdit, "dial_min_val_" + str(id_))
                    encoder_button._max = self.findChild(QLineEdit, "dial_max_val_" + str(id_))
                    encoder_button._val_set = self.findChild(QLineEdit, "dial_val_set_" + str(id_))
                    encoder_button._pop_up_dialog = PopupEditDialog()

                    encoder_button._control_param_name.setText(encoder_button.name)
                    encoder_button._min.setText(str(encoder_button.min_value))
                    encoder_button._max.setText(str(encoder_button.max_value))
                    encoder_button._dial.setValue(encoder_button.default_value)
                    d_validator = QDoubleValidator(encoder_button.min_value, encoder_button.max_value, 20)
                    encoder_button._validator = d_validator
                    encoder_button._val_set.setValidator(d_validator)

                    encoder_button._dial.valueChanged.connect(encoder_button.on_dial_changed_value_cb)
                    encoder_button._val_set.returnPressed.connect(encoder_button.on_val_set_return_pressed_cb)
                    encoder_button._min.returnPressed.connect(encoder_button.on_range_edited_cb)
                    encoder_button._max.returnPressed.connect(encoder_button.on_range_edited_cb)

                    default_values[encoder_button.name] = encoder_button.default_value

        for button in buttons:
            if button.identifier in MidiDialog.BUTTON_TOGGLE_RAW_ID_TO_ID.keys():
                id_ = MidiDialog.BUTTON_TOGGLE_RAW_ID_TO_ID.get(button.identifier)
                if id_ is not None:
                    button._button_toggle = self.findChild(QPushButton, "button_toggle_" + str(id_))
                    button._button_toggle.setText(button.name)

                    button._button_toggle.clicked.connect(button.on_clicked_cb)

        for slider_ctrl in sliders:
            if slider_ctrl.identifier in MidiDialog.SLIDER_RAW_ID_TO_ID.keys():
                id_ = MidiDialog.SLIDER_RAW_ID_TO_ID.get(slider_ctrl.identifier)
                if id_ is not None:
                    slider_ctrl._slider = self.findChild(QSlider, "slider_" + str(id_))
                    slider_ctrl._control_param_name = self.findChild(QLabel, "control_param_name_" + str(id_))
                    slider_ctrl._min = self.findChild(QLineEdit, "min_val_" + str(id_))
                    slider_ctrl._max = self.findChild(QLineEdit, "max_val_" + str(id_))
                    slider_ctrl._val_set = self.findChild(QLineEdit, "val_set_" + str(id_))
                    slider_ctrl._pop_up_dialog = PopupEditDialog()

                    slider_ctrl._control_param_name.setText(slider_ctrl.name)
                    slider_ctrl._min.setText(str(slider_ctrl.min_value))
                    slider_ctrl._max.setText(str(slider_ctrl.max_value))
                    slider_ctrl._slider.setValue(slider_ctrl.default_value)
                    d_validator = QDoubleValidator(slider_ctrl.min_value, slider_ctrl.max_value, 20)
                    slider_ctrl._validator = d_validator
                    slider_ctrl._val_set.setValidator(d_validator)

                    slider_ctrl._slider.valueChanged.connect(slider_ctrl.on_slider_changed_value_cb)
                    slider_ctrl._val_set.returnPressed.connect(slider_ctrl.on_val_set_return_pressed_cb)
                    slider_ctrl._min.returnPressed.connect(slider_ctrl.on_range_edited_cb)
                    slider_ctrl._max.returnPressed.connect(slider_ctrl.on_range_edited_cb)

                    default_values[slider_ctrl.name] = slider_ctrl.default_value

        for view_ctrl in views:
            param_label = QtWidgets.QLabel(view_ctrl.name)
            view_ctrl._label = param_label
            param_label.setObjectName(view_ctrl.name)
            self.vertical_layout.addWidget(param_label)

        set_defaults_button._sliders = sliders
        set_defaults_button._encoder_buttons = encoder_buttons
        set_defaults_button._button = self.default_button
        set_defaults_button._button.clicked.connect(lambda: set_defaults_button.on_default_button_clicked_cb(default_values))

    def closeEvent(self, event):
        quit_msg = "Are you sure you want to exit?"

        reply = QtWidgets.QMessageBox().question(self, 'Message', quit_msg, QtWidgets.QMessageBox.Yes,
                                                 QtWidgets.QMessageBox.No)
        if reply == QtWidgets.QMessageBox.Yes:
            self.close()
        else:
            event.ignore()

    def keyPressEvent(self, key_event):
        if not key_event.key() == Qt.Key_Escape:
            QDialog.keyPressEvent(self, key_event)

    def update_board_connection_status(self):
        is_board_plugged = midi_utility.is_board_plugged()

        if not is_board_plugged:
            self.board_status_label.setText("Board Status: Disconnected")
        else:
            if self.board_status_label.text() == "Board Status":  # Initial text.
                self.board_status_label.setText("Board Status: Connected")

            if self.board_status_label.text() == "Board Status: Disconnected":
                self.board_status_label.setText("Board Status: Connected after disconnected. "
                                                "Please restart the program!")
