from PySide.QtCore import *
from PySide.QtGui import *

class SettingsManager(QSettings):
    def __init__(self, settings_path):
        super(SettingsManager, self).__init__(settings_path, QSettings.IniFormat)

    def save(self, ui_object):
        if type(ui_object) == QLineEdit:
            self.setValue(ui_object.objectName(), ui_object.text())
        elif type(ui_object) == QTextEdit:
            self.setValue(ui_object.objectName(), ui_object.toPlainText())

    def load(self, ui_object):
        if type(ui_object) in [QLineEdit, QTextEdit]:
            value = self.value(ui_object.objectName())
            if value:
                ui_object.setText(value)