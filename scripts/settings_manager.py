# Copyright 2020 Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
        elif type(ui_object) == QCheckBox:
            self.setValue(ui_object.objectName(), int(ui_object.isChecked()))

    def load(self, ui_object):
        if type(ui_object) in [QLineEdit, QTextEdit]:
            value = self.value(ui_object.objectName())
            if value:
                ui_object.setText(value)
        elif type(ui_object) == QCheckBox:
            value = self.value(ui_object.objectName())
            if not value or value == '0':
                ui_object.setCheckState(Qt.Unchecked)
            else:
                ui_object.setCheckState(Qt.Checked)