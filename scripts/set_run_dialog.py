#! /usr/bin/env python
#  -*- coding: utf-8 -*-
import sys
import os
import rospy
from PySide.QtGui import *
from PySide.QtCore import *
from ui.ui_set_run_dialog import Ui_set_run_dialog
from controller.vehicle_manual_controller import VehicleManualController
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

        self.vehicle_manual_controller = VehicleManualController()

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

    def stop_button_clicked(self):
        self.vehicle_manual_controller.set_velocity_kmph(0.0, 0.0)

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