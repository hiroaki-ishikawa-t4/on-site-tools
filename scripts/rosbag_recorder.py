#! /usr/bin/env python
#  -*- coding: utf-8 -*-
#
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

import sys
import os
from PySide.QtGui import *
from PySide.QtCore import *
from ui.ui_rosbag_recorder import Ui_rosbag_recorder
from controller.rosbag_controller import RosbagController
from controller.candump_controller import CandumpController
from lock_manager import LockManager
from settings_manager import SettingsManager

CONFIG_FILE = 'rosbag_recorder.ini'
LOCK_PATH = '/tmp/rosbag_recorder.lock'

class RosbagRecorderDialog(QDialog):
    def __init__(self, parent=None):
        super(RosbagRecorderDialog, self).__init__(parent)
        self.ui = Ui_rosbag_recorder()
        self.ui.setupUi(self)

        self.rosbag_controller = RosbagController()
        self.candump_controller = CandumpController()
        self.found_candump = self.candump_controller.check_candump_exist()

        self.ui.start_button.clicked.connect(self.start_button_clicked)
        self.ui.stop_button.clicked.connect(self.stop_button_clicked)
        self.ui.choose_button.clicked.connect(self.choose_button_clicked)
        self.ui.description_edit.textChanged.connect(self.description_edit_text_changed)

        self.ui.start_button.setEnabled(True)
        self.ui.stop_button.setEnabled(False)
        self.load_ui_info()

        # Set window to stay top
        self.setWindowFlags(Qt.WindowStaysOnTopHint)

    def choose_button_clicked(self):
        # Choose save root directory
        path = QFileDialog.getExistingDirectory(self)
        if path != '':
            self.ui.root_dir_edit.setText(path)

    def start_button_clicked(self):
        root_dir = self.ui.root_dir_edit.text()
        title = self.ui.title_edit.text()
        description = self.ui.description_edit.toPlainText()
        options = self.ui.command_option_edit.toPlainText()

        # Check previous directory existence
        save_dir = (root_dir+'/').replace('//', '/') + title
        if os.path.exists(save_dir):
            QMessageBox.warning(None, 'Error', 'Directory is already exists. Please change title.', QMessageBox.Ok)
            return
        
        # Check option
        if options == '':
            ret = QMessageBox.information(None, 'Info', "You didn't specified any option. Is it means you want to record everything?", QMessageBox.Yes | QMessageBox.No)
            if ret == QMessageBox.Yes:
                options = '-a'

        # Start rosbag record
        if not self.rosbag_controller.start_record(root_dir, title, description, options):
            QMessageBox.warning(None, 'Warning', 'Failed to start rosbag record!', QMessageBox.Ok)
            return

        # Start candump
        if self.found_candump and self.ui.candump_checkbox.isChecked():
            if not self.candump_controller.start(self.ui.can_device_edit.text(), save_dir):
                QMessageBox.warning(None, 'Warning', 'Failed to start candump. Continue...', QMessageBox.Ok)

        self.ui.start_button.setEnabled(False)
        self.ui.stop_button.setEnabled(True)
        self.setStyleSheet('background-color: hotpink;')

    def stop_button_clicked(self):
        # Finish rosbag
        if not self.rosbag_controller.finish_record():
            QMessageBox.warning(None, 'Error', 'Failed to stop rosbag record!', QMessageBox.Ok)
            return

        # Finish candump
        if self.candump_controller.is_working():
            if not self.candump_controller.finish():
                QMessageBox.warning(None, 'Error', 'Failed to stop candump!', QMessageBox.Ok)

        self.ui.start_button.setEnabled(True)
        self.ui.stop_button.setEnabled(False)
        self.setStyleSheet('')

    def description_edit_text_changed(self):
        self.rosbag_controller.update_description(self.ui.description_edit.toPlainText())

    def closeEvent(self, event):
        self.save_ui_info()

    def load_ui_info(self):
        settings = SettingsManager(CONFIG_FILE)
        settings.load(self.ui.root_dir_edit)
        settings.load(self.ui.title_edit)
        settings.load(self.ui.description_edit)
        settings.load(self.ui.command_option_edit)
        settings.load(self.ui.can_device_edit)
        settings.load(self.ui.candump_checkbox)

        # Set default value
        if self.ui.can_device_edit.text() == '':
            self.ui.can_device_edit.setText('can0')

        self.ui.candump_checkbox.setEnabled(self.found_candump)

    def save_ui_info(self):
        settings = SettingsManager(CONFIG_FILE)
        settings.save(self.ui.root_dir_edit)
        settings.save(self.ui.title_edit)
        settings.save(self.ui.description_edit)
        settings.save(self.ui.command_option_edit)
        settings.save(self.ui.can_device_edit)
        settings.save(self.ui.candump_checkbox)

if __name__=='__main__':
    app = QApplication(sys.argv)

    lock_manager = LockManager(LOCK_PATH)
    if not lock_manager.get_lock():
        QMessageBox.warning(None, 'Warning', 'Another same process is running. If not, please delete lock file [ %s ].' % lock_manager.get_lock_path(), QMessageBox.Ok)
        sys.exit(1)
    
    win = RosbagRecorderDialog()
    win.show()
    ret = app.exec_()
    lock_manager.release_lock()
    sys.exit(ret)