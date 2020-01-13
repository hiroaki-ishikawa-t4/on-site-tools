#! /usr/bin/env python
#  -*- coding: utf-8 -*-
import sys
import os
import rospy
from PySide.QtGui import *
from PySide.QtCore import *
from ui.ui_waypoint_recoder import Ui_waypoint_recorder
from controller.waypoint_saver_controller import WaypointSaverController
from lock_manager import LockManager
from settings_manager import SettingsManager

CONFIG_FILE = 'waypoint_recoder.ini'
LOCK_PATH = '/tmp/waypoint_recoder.lock'

class WaypointRecorderDialog(QDialog):
    def __init__(self, parent=None):
        super(WaypointRecorderDialog, self).__init__(parent)
        self.ui = Ui_waypoint_recorder()
        self.ui.setupUi(self)

        self.ui.start_button.clicked.connect(self.start_button_clicked)
        self.ui.stop_button.clicked.connect(self.stop_button_clicked)
        self.ui.choose_button.clicked.connect(self.choose_button_clicked)

        self.ui.start_button.setEnabled(True)
        self.ui.stop_button.setEnabled(False)
        self.load_ui_info()

        self.waypoint_saver_controller = WaypointSaverController()

        # Set window to stay top
        self.setWindowFlags(Qt.WindowStaysOnTopHint)

    def choose_button_clicked(self):
        # Choose save root directory
        path = QFileDialog.getSaveFileName(self, 'CSV Files (*.csv)')[0]
        self.ui.save_path_edit.setText(path)

    def start_button_clicked(self):
        save_path = self.ui.save_path_edit.text()

        # Check previous directory existence
        if os.path.exists(save_path):
            QMessageBox.warning(None, 'Error', 'File is already exists. Please change the name.', QMessageBox.Ok)
            return
        
        # Start record
        if not self.waypoint_saver_controller.start_record(save_path):
            QMessageBox.warning(None, 'Warning', 'Failed to start waypoint record!', QMessageBox.Ok)
            return

        self.ui.start_button.setEnabled(False)
        self.ui.stop_button.setEnabled(True)

    def stop_button_clicked(self):
        if not self.waypoint_saver_controller.finish_record():
            QMessageBox.warning(None, 'Error', 'Failed to stop waypoint record!', QMessageBox.Ok)
            return
        self.ui.start_button.setEnabled(True)
        self.ui.stop_button.setEnabled(False)

    def closeEvent(self, event):
        self.save_ui_info()

    def load_ui_info(self):
        settings = SettingsManager(CONFIG_FILE)
        settings.load(self.ui.save_path_edit)

    def save_ui_info(self):
        settings = SettingsManager(CONFIG_FILE)
        settings.save(self.ui.save_path_edit)

if __name__=='__main__':
    rospy.init_node('waypoint_recorder')

    app = QApplication(sys.argv)

    lock_manager = LockManager(LOCK_PATH)
    if not lock_manager.get_lock():
        QMessageBox.warning(None, 'Warning', 'Another same process is running. If not, please delete lock file [ %s ].' % lock_manager.get_lock_path(), QMessageBox.Ok)
        sys.exit(1)
    
    win = WaypointRecorderDialog()
    win.show()
    ret = app.exec_()
    lock_manager.release_lock()
    sys.exit(ret)