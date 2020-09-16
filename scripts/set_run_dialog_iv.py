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
import rospy
from PySide.QtGui import *
from PySide.QtCore import *
from ui.ui_set_run_dialog import Ui_set_run_dialog
from controller.vehicle_manual_controller_iv import VehicleManualControllerIv
from lock_manager import LockManager

LOCK_PATH = '/tmp/set_run_dialog.lock'

class SetRunDialog(QDialog):
    def __init__(self, parent=None):
        super(SetRunDialog, self).__init__(parent)
        self.ui = Ui_set_run_dialog()
        self.ui.setupUi(self)

        self.ui.start_button.clicked.connect(self.start_button_clicked)
        self.ui.stop_button.clicked.connect(self.stop_button_clicked)

        # Init UI value
        self.ui.linear_edit.setText('0.0')
        self.ui.angular_edit.setText('0.0')

        self.vehicle_manual_controller = VehicleManualControllerIv()

        # Set window to stay top
        self.setWindowFlags(Qt.WindowStaysOnTopHint)

    def start_button_clicked(self):
        # Get value from UI
        try:
            linear_kmph = float(self.ui.linear_edit.text())
            steer_angle = float(self.ui.angular_edit.text())
        except ValueError:
            QMessageBox.warning(None, 'Warning', 'Input correct value!', QMessageBox.Ok)
            return
        # Set value
        self.vehicle_manual_controller.set_velocity_kmph(linear_kmph, steer_angle)
        # Transfer focus
        self.ui.stop_button.setFocus()

    def stop_button_clicked(self):
        # Set value
        self.vehicle_manual_controller.set_velocity_kmph(0.0, 0.0)
        # Transfer focus
        self.ui.start_button.setFocus()

    def closeEvent(self, event):
        # Stop vehicle for sure
        self.vehicle_manual_controller.stop_now()

if __name__=='__main__':
    rospy.init_node('set_run_dialog')

    app = QApplication(sys.argv)

    lock_manager = LockManager(LOCK_PATH)
    if not lock_manager.get_lock():
        QMessageBox.warning(None, 'Warning', 'Another same process is running. If not, please delete lock file [ %s ].' % lock_manager.get_lock_path(), QMessageBox.Ok)
        sys.exit(1)
    
    win = SetRunDialog()
    win.show()
    ret = app.exec_()
    lock_manager.release_lock()
    sys.exit(ret)