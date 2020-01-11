#! /usr/bin/env python
#  -*- coding: utf-8 -*-
import sys
import os
import rospy
from PySide.QtGui import *
from PySide.QtCore import *
from ui.ui_rosbag_recorder import Ui_rosbag_recorder
from controller.rosbag_controller import RosbagController
from lock_manager import LockManager
from settings_manager import SettingsManager

CONFIG_FILE = 'rosbag_recoder.ini'

class RosbagRecorderDialog(QDialog):
    def __init__(self, parent=None):
        super(RosbagRecorderDialog, self).__init__(parent)
        self.ui = Ui_rosbag_recorder()
        self.ui.setupUi(self)

        self.ui.start_button.clicked.connect(self.start_button_clicked)
        self.ui.stop_button.clicked.connect(self.stop_button_clicked)
        self.ui.choose_button.clicked.connect(self.choose_button_clicked)

        self.ui.start_button.setEnabled(True)
        self.ui.stop_button.setEnabled(False)
        self.load_ui_info()

        self.rosbag_controller = RosbagController()

        # Set window to stay top
        self.setWindowFlags(Qt.WindowStaysOnTopHint)

    def choose_button_clicked(self):
        # Choose save root directory
        path = QFileDialog.getExistingDirectory(self)
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

        if not self.rosbag_controller.start_record(root_dir, title, description, options):
            QMessageBox.warning(None, 'Warning', 'Failed to start rosbag record!', QMessageBox.Ok)
        self.ui.start_button.setEnabled(False)
        self.ui.stop_button.setEnabled(True)

    def stop_button_clicked(self):
        if not self.rosbag_controller.finish_record():
            QMessageBox.warning(None, 'Error', 'Failed to stop rosbag record!', QMessageBox.Ok)
            return
        self.ui.start_button.setEnabled(True)
        self.ui.stop_button.setEnabled(False)

    def closeEvent(self, event):
        self.save_ui_info()

    def load_ui_info(self):
        settings = SettingsManager(CONFIG_FILE)
        settings.load(self.ui.root_dir_edit)
        settings.load(self.ui.title_edit)
        settings.load(self.ui.description_edit)
        settings.load(self.ui.command_option_edit)

    def save_ui_info(self):
        settings = SettingsManager(CONFIG_FILE)
        settings.save(self.ui.root_dir_edit)
        settings.save(self.ui.title_edit)
        settings.save(self.ui.description_edit)
        settings.save(self.ui.command_option_edit)

if __name__=='__main__':
    rospy.init_node('rosbag_recorder')

    app = QApplication(sys.argv)

    if not LockManager.get_lock():
        QMessageBox.warning(None, 'Warning', 'Another same process is running. If not, please delete lock file [ %s ].' % LockManager.get_lock_path(), QMessageBox.Ok)
        sys.exit(1)
    
    win = RosbagRecorderDialog()
    win.show()
    ret = app.exec_()
    LockManager.release_lock()
    sys.exit(ret)