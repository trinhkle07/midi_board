# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'popup_dialog.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_popup_dialog(object):
    def setupUi(self, popup_dialog):
        popup_dialog.setObjectName("popup_dialog")
        popup_dialog.resize(369, 138)
        self.verticalLayout = QtWidgets.QVBoxLayout(popup_dialog)
        self.verticalLayout.setObjectName("verticalLayout")
        self.popup_question_label = QtWidgets.QLabel(popup_dialog)
        self.popup_question_label.setObjectName("popup_question_label")
        self.verticalLayout.addWidget(self.popup_question_label)
        self.popup_form_layout = QtWidgets.QFormLayout()
        self.popup_form_layout.setObjectName("popup_form_layout")
        self.popup_max_label = QtWidgets.QLabel(popup_dialog)
        self.popup_max_label.setObjectName("popup_max_label")
        self.popup_form_layout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.popup_max_label)
        self.popup_max_val = QtWidgets.QLineEdit(popup_dialog)
        self.popup_max_val.setObjectName("popup_max_val")
        self.popup_form_layout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.popup_max_val)
        self.popup_min_label = QtWidgets.QLabel(popup_dialog)
        self.popup_min_label.setObjectName("popup_min_label")
        self.popup_form_layout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.popup_min_label)
        self.popup_min_val = QtWidgets.QLineEdit(popup_dialog)
        self.popup_min_val.setObjectName("popup_min_val")
        self.popup_form_layout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.popup_min_val)
        self.verticalLayout.addLayout(self.popup_form_layout)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.popup_err_msg = QtWidgets.QLabel(popup_dialog)
        self.popup_err_msg.setObjectName("popup_err_msg")
        self.horizontalLayout.addWidget(self.popup_err_msg)
        self.popup_answer_box = QtWidgets.QDialogButtonBox(popup_dialog)
        self.popup_answer_box.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.popup_answer_box.setObjectName("popup_answer_box")
        self.horizontalLayout.addWidget(self.popup_answer_box)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.popup_max_label.setBuddy(self.popup_max_val)
        self.popup_min_label.setBuddy(self.popup_min_val)

        self.retranslateUi(popup_dialog)
        self.popup_answer_box.accepted.connect(popup_dialog.accept)
        self.popup_answer_box.rejected.connect(popup_dialog.reject)
        QtCore.QMetaObject.connectSlotsByName(popup_dialog)

    def retranslateUi(self, popup_dialog):
        _translate = QtCore.QCoreApplication.translate
        popup_dialog.setWindowTitle(_translate("popup_dialog", "New range"))
        self.popup_question_label.setText(_translate("popup_dialog", "What are min and max values you would like to set ?"))
        self.popup_max_label.setText(_translate("popup_dialog", "Maximum"))
        self.popup_max_val.setPlaceholderText(_translate("popup_dialog", "Max value ?"))
        self.popup_min_label.setText(_translate("popup_dialog", "Minimum"))
        self.popup_min_val.setPlaceholderText(_translate("popup_dialog", "Min value ?"))
        self.popup_err_msg.setText(_translate("popup_dialog", "<html><head/><body><p><br/></p></body></html>"))

