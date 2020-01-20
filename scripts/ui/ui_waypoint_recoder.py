# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file './ui/waypoint_recoder.ui'
#
# Created: Sun Jan 19 20:52:10 2020
#      by: pyside-uic 0.2.15 running on PySide 1.2.2
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_waypoint_recorder(object):
    def setupUi(self, waypoint_recorder):
        waypoint_recorder.setObjectName("waypoint_recorder")
        waypoint_recorder.resize(400, 188)
        self.verticalLayout = QtGui.QVBoxLayout(waypoint_recorder)
        self.verticalLayout.setObjectName("verticalLayout")
        self.gridLayout = QtGui.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.save_path_edit = QtGui.QLineEdit(waypoint_recorder)
        self.save_path_edit.setObjectName("save_path_edit")
        self.gridLayout.addWidget(self.save_path_edit, 0, 0, 1, 1)
        self.choose_button = QtGui.QPushButton(waypoint_recorder)
        self.choose_button.setObjectName("choose_button")
        self.gridLayout.addWidget(self.choose_button, 0, 1, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout)
        self.gridLayout_2 = QtGui.QGridLayout()
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.start_button = QtGui.QPushButton(waypoint_recorder)
        self.start_button.setObjectName("start_button")
        self.gridLayout_2.addWidget(self.start_button, 0, 0, 1, 1)
        self.stop_button = QtGui.QPushButton(waypoint_recorder)
        self.stop_button.setObjectName("stop_button")
        self.gridLayout_2.addWidget(self.stop_button, 0, 1, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout_2)

        self.retranslateUi(waypoint_recorder)
        QtCore.QMetaObject.connectSlotsByName(waypoint_recorder)

    def retranslateUi(self, waypoint_recorder):
        waypoint_recorder.setWindowTitle(QtGui.QApplication.translate("waypoint_recorder", "waypoint_recoder", None, QtGui.QApplication.UnicodeUTF8))
        self.choose_button.setText(QtGui.QApplication.translate("waypoint_recorder", "Choose", None, QtGui.QApplication.UnicodeUTF8))
        self.start_button.setText(QtGui.QApplication.translate("waypoint_recorder", "Start", None, QtGui.QApplication.UnicodeUTF8))
        self.stop_button.setText(QtGui.QApplication.translate("waypoint_recorder", "Stop", None, QtGui.QApplication.UnicodeUTF8))

