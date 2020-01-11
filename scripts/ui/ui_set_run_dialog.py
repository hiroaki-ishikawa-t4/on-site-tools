# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file './ui/set_run_dialog.ui'
#
# Created: Sat Jan 11 20:45:10 2020
#      by: pyside-uic 0.2.15 running on PySide 1.2.2
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_set_run_dialog(object):
    def setupUi(self, set_run_dialog):
        set_run_dialog.setObjectName("set_run_dialog")
        set_run_dialog.resize(296, 119)
        self.gridLayoutWidget = QtGui.QWidget(set_run_dialog)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(10, 10, 278, 98))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtGui.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.start_button = QtGui.QPushButton(self.gridLayoutWidget)
        self.start_button.setObjectName("start_button")
        self.gridLayout.addWidget(self.start_button, 2, 0, 1, 1)
        self.stop_button = QtGui.QPushButton(self.gridLayoutWidget)
        self.stop_button.setObjectName("stop_button")
        self.gridLayout.addWidget(self.stop_button, 2, 1, 1, 1)
        self.angular_edit = QtGui.QLineEdit(self.gridLayoutWidget)
        self.angular_edit.setEnabled(False)
        self.angular_edit.setObjectName("angular_edit")
        self.gridLayout.addWidget(self.angular_edit, 1, 1, 1, 1)
        self.label = QtGui.QLabel(self.gridLayoutWidget)
        self.label.setObjectName("label")
        self.gridLayout.addWidget(self.label, 0, 0, 1, 1)
        self.linear_edit = QtGui.QLineEdit(self.gridLayoutWidget)
        self.linear_edit.setObjectName("linear_edit")
        self.gridLayout.addWidget(self.linear_edit, 0, 1, 1, 1)
        self.label_2 = QtGui.QLabel(self.gridLayoutWidget)
        self.label_2.setObjectName("label_2")
        self.gridLayout.addWidget(self.label_2, 1, 0, 1, 1)

        self.retranslateUi(set_run_dialog)
        QtCore.QMetaObject.connectSlotsByName(set_run_dialog)

    def retranslateUi(self, set_run_dialog):
        set_run_dialog.setWindowTitle(QtGui.QApplication.translate("set_run_dialog", "Dialog", None, QtGui.QApplication.UnicodeUTF8))
        self.start_button.setText(QtGui.QApplication.translate("set_run_dialog", "Start", None, QtGui.QApplication.UnicodeUTF8))
        self.stop_button.setText(QtGui.QApplication.translate("set_run_dialog", "Stop", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("set_run_dialog", "Linear [km/h]:", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("set_run_dialog", "Steering angle [Deg]:", None, QtGui.QApplication.UnicodeUTF8))

