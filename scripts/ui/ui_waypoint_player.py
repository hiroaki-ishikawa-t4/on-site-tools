# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file './ui/waypoint_player.ui'
#
# Created: Sun Jan 19 20:52:10 2020
#      by: pyside-uic 0.2.15 running on PySide 1.2.2
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_waypoint_player(object):
    def setupUi(self, waypoint_player):
        waypoint_player.setObjectName("waypoint_player")
        waypoint_player.resize(400, 198)
        self.verticalLayout = QtGui.QVBoxLayout(waypoint_player)
        self.verticalLayout.setObjectName("verticalLayout")
        self.gridLayout = QtGui.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.load_path_edit = QtGui.QLineEdit(waypoint_player)
        self.load_path_edit.setObjectName("load_path_edit")
        self.gridLayout.addWidget(self.load_path_edit, 0, 0, 1, 1)
        self.choose_button = QtGui.QPushButton(waypoint_player)
        self.choose_button.setObjectName("choose_button")
        self.gridLayout.addWidget(self.choose_button, 0, 1, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout)
        self.gridLayout_2 = QtGui.QGridLayout()
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.label = QtGui.QLabel(waypoint_player)
        self.label.setObjectName("label")
        self.gridLayout_2.addWidget(self.label, 0, 0, 1, 1)
        self.override_velocity_edit = QtGui.QLineEdit(waypoint_player)
        self.override_velocity_edit.setObjectName("override_velocity_edit")
        self.gridLayout_2.addWidget(self.override_velocity_edit, 0, 1, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout_2)
        self.gridLayout_3 = QtGui.QGridLayout()
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.start_button = QtGui.QPushButton(waypoint_player)
        self.start_button.setObjectName("start_button")
        self.gridLayout_3.addWidget(self.start_button, 0, 0, 1, 1)
        self.stop_button = QtGui.QPushButton(waypoint_player)
        self.stop_button.setObjectName("stop_button")
        self.gridLayout_3.addWidget(self.stop_button, 0, 1, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout_3)

        self.retranslateUi(waypoint_player)
        QtCore.QMetaObject.connectSlotsByName(waypoint_player)

    def retranslateUi(self, waypoint_player):
        waypoint_player.setWindowTitle(QtGui.QApplication.translate("waypoint_player", "waypoint_player", None, QtGui.QApplication.UnicodeUTF8))
        self.choose_button.setText(QtGui.QApplication.translate("waypoint_player", "Choose", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("waypoint_player", "Override velocity [km/h]:", None, QtGui.QApplication.UnicodeUTF8))
        self.start_button.setText(QtGui.QApplication.translate("waypoint_player", "Start", None, QtGui.QApplication.UnicodeUTF8))
        self.stop_button.setText(QtGui.QApplication.translate("waypoint_player", "Stop", None, QtGui.QApplication.UnicodeUTF8))

