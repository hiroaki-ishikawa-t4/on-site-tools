#! /usr/bin/env python
#  -*- coding: utf-8 -*-
import sys
import os
import rospy
from PySide.QtGui import *
from PySide.QtCore import *
from ui.ui_waypoint_player import Ui_waypoint_player
from controller.waypoint_loader_controller import WaypointLoaderController
from lock_manager import LockManager
from settings_manager import SettingsManager

CONFIG_FILE = 'waypoint_player.ini'
LOCK_PATH = '/tmp/waypoint_player.lock'

class WaypointPlayerDialog(QDialog):
    def __init__(self, parent=None):
        super(WaypointPlayerDialog, self).__init__(parent)
        self.ui = Ui_waypoint_player()
        self.ui.setupUi(self)

        self.ui.start_button.clicked.connect(self.start_button_clicked)
        self.ui.stop_button.clicked.connect(self.stop_button_clicked)
        self.ui.choose_button.clicked.connect(self.choose_button_clicked)

        self.ui.start_button.setEnabled(True)
        self.ui.stop_button.setEnabled(False)
        self.load_ui_info()

        self.waypoint_loader_controller = WaypointLoaderController()

        # Set window to stay top
        self.setWindowFlags(Qt.WindowStaysOnTopHint)

    def choose_button_clicked(self):
        # Choose save root directory
        path = QFileDialog.getOpenFileName(self, 'CSV Files (*.csv)')[0]
        self.ui.load_path_edit.setText(path)

    def start_button_clicked(self):
        load_path = self.ui.load_path_edit.text()

        override_velocity_str = self.ui.override_velocity_edit.text()
        if override_velocity_str != '':
            override_enable = True
            try:
                override_velocity = float(override_velocity_str)
            except ValueError:
                QMessageBox.warning(None, 'Error', 'Please input correct value for override velocity.', QMessageBox.Ok)
                return
        else:
            override_enable = False
            override_velocity = 0.0

        # Check file existence
        if not os.path.exists(load_path):
            QMessageBox.warning(None, 'Error', 'File is not exists.', QMessageBox.Ok)
            return
        
        # Start play
        if not self.waypoint_loader_controller.start(load_path, override_enable, override_velocity):
            QMessageBox.warning(None, 'Warning', 'Failed to start waypoint play!', QMessageBox.Ok)
            return

        self.ui.start_button.setEnabled(False)
        self.ui.stop_button.setEnabled(True)

    def stop_button_clicked(self):
        if not self.waypoint_loader_controller.stop():
            QMessageBox.warning(None, 'Error', 'Failed to stop waypoint play!', QMessageBox.Ok)
            return
        self.ui.start_button.setEnabled(True)
        self.ui.stop_button.setEnabled(False)

    def closeEvent(self, event):
        self.save_ui_info()

    def load_ui_info(self):
        settings = SettingsManager(CONFIG_FILE)
        settings.load(self.ui.load_path_edit)
        settings.load(self.ui.override_velocity_edit)

    def save_ui_info(self):
        settings = SettingsManager(CONFIG_FILE)
        settings.save(self.ui.load_path_edit)
        settings.save(self.ui.override_velocity_edit)

if __name__=='__main__':
    rospy.init_node('waypoint_player')

    app = QApplication(sys.argv)

    lock_manager = LockManager(LOCK_PATH)
    if not lock_manager.get_lock():
        QMessageBox.warning(None, 'Warning', 'Another same process is running. If not, please delete lock file [ %s ].' % lock_manager.get_lock_path(), QMessageBox.Ok)
        sys.exit(1)
    
    win = WaypointPlayerDialog()
    win.show()
    ret = app.exec_()
    lock_manager.release_lock()
    sys.exit(ret)