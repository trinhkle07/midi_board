from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import popup_dialog_ui


class PopupEditDialog(QDialog, popup_dialog_ui.Ui_popup_dialog):

    DEFAULT_NAME = "MyPopupEditDialog"

    def __init__(self, curr_min_val=None, curr_max_val=None):
        QDialog.__init__(self)
        self.setupUi(self)

        self._curr_min = curr_min_val
        self._curr_max = curr_max_val
        self._new_min = None
        self._new_max = None

        self.set_placeholder_text()

        self.popup_min_val.setValidator(QDoubleValidator())
        self.popup_max_val.setValidator(QDoubleValidator())

        self.popup_min_val.editingFinished.connect(self.edit_min_finished_cb)
        self.popup_max_val.editingFinished.connect(self.edit_max_finished_cb)

    def set_placeholder_text(self):
        self.popup_min_val.setPlaceholderText(str(self._curr_min))
        self.popup_max_val.setPlaceholderText(str(self._curr_max))

    def edit_min_finished_cb(self):
        self._new_min = float(self.popup_min_val.text())

    def edit_max_finished_cb(self):
        self._new_max = float(self.popup_max_val.text())

    @property
    def curr_max(self):
        """Get the current maximum value of the slider."""
        return self._curr_max

    @curr_max.setter
    def curr_max(self, val):
        """Set the current maximum value of the slider."""
        self._curr_max = val
        self.set_placeholder_text()

    @property
    def curr_min(self):
        """Get the current minimum value of the slider."""
        return self._curr_min

    @curr_min.setter
    def curr_min(self, val):
        """Set the current minimum value of the slider."""
        self._curr_min = val
        self.set_placeholder_text()

    @property
    def new_max(self):
        """Get the new maximum value of the slider."""
        return self._new_max

    @new_max.setter
    def new_max(self, val):
        """Set the new maximum value of the slider."""
        self._new_max = val

    @property
    def new_min(self):
        """Get the new minimum value of the slider."""
        return self._new_min

    @new_min.setter
    def new_min(self, val):
        """Set the new minimum value of the slider."""
        self._new_min = val

