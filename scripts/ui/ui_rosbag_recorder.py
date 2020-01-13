# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file './ui/rosbag_recorder.ui'
#
# Created: Tue Jan 14 00:59:55 2020
#      by: pyside-uic 0.2.15 running on PySide 1.2.2
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_rosbag_recorder(object):
    def setupUi(self, rosbag_recorder):
        rosbag_recorder.setObjectName("rosbag_recorder")
        rosbag_recorder.resize(522, 492)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(rosbag_recorder.sizePolicy().hasHeightForWidth())
        rosbag_recorder.setSizePolicy(sizePolicy)
        self.verticalLayout = QtGui.QVBoxLayout(rosbag_recorder)
        self.verticalLayout.setObjectName("verticalLayout")
        self.gridLayout_4 = QtGui.QGridLayout()
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.label_3 = QtGui.QLabel(rosbag_recorder)
        self.label_3.setObjectName("label_3")
        self.gridLayout_4.addWidget(self.label_3, 0, 0, 1, 1)
        self.root_dir_edit = QtGui.QLineEdit(rosbag_recorder)
        self.root_dir_edit.setObjectName("root_dir_edit")
        self.gridLayout_4.addWidget(self.root_dir_edit, 0, 1, 1, 1)
        self.choose_button = QtGui.QPushButton(rosbag_recorder)
        self.choose_button.setObjectName("choose_button")
        self.gridLayout_4.addWidget(self.choose_button, 0, 2, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout_4)
        self.groupBox = QtGui.QGroupBox(rosbag_recorder)
        self.groupBox.setMinimumSize(QtCore.QSize(0, 200))
        self.groupBox.setObjectName("groupBox")
        self.gridLayoutWidget = QtGui.QWidget(self.groupBox)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(0, 20, 491, 181))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtGui.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setSizeConstraint(QtGui.QLayout.SetDefaultConstraint)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.label = QtGui.QLabel(self.gridLayoutWidget)
        self.label.setObjectName("label")
        self.gridLayout.addWidget(self.label, 1, 0, 1, 1)
        self.title_edit = QtGui.QLineEdit(self.gridLayoutWidget)
        self.title_edit.setObjectName("title_edit")
        self.gridLayout.addWidget(self.title_edit, 0, 1, 1, 1)
        self.label_2 = QtGui.QLabel(self.gridLayoutWidget)
        self.label_2.setObjectName("label_2")
        self.gridLayout.addWidget(self.label_2, 0, 0, 1, 1)
        self.description_edit = QtGui.QTextEdit(self.gridLayoutWidget)
        self.description_edit.setObjectName("description_edit")
        self.gridLayout.addWidget(self.description_edit, 1, 1, 1, 1)
        self.verticalLayout.addWidget(self.groupBox)
        self.groupBox_2 = QtGui.QGroupBox(rosbag_recorder)
        self.groupBox_2.setMinimumSize(QtCore.QSize(0, 150))
        self.groupBox_2.setObjectName("groupBox_2")
        self.command_option_edit = QtGui.QTextEdit(self.groupBox_2)
        self.command_option_edit.setGeometry(QtCore.QRect(0, 20, 501, 131))
        self.command_option_edit.setObjectName("command_option_edit")
        self.verticalLayout.addWidget(self.groupBox_2)
        self.gridLayout_2 = QtGui.QGridLayout()
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.stop_button = QtGui.QPushButton(rosbag_recorder)
        self.stop_button.setObjectName("stop_button")
        self.gridLayout_2.addWidget(self.stop_button, 0, 1, 1, 1)
        self.start_button = QtGui.QPushButton(rosbag_recorder)
        self.start_button.setObjectName("start_button")
        self.gridLayout_2.addWidget(self.start_button, 0, 0, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout_2)

        self.retranslateUi(rosbag_recorder)
        QtCore.QMetaObject.connectSlotsByName(rosbag_recorder)

    def retranslateUi(self, rosbag_recorder):
        rosbag_recorder.setWindowTitle(QtGui.QApplication.translate("rosbag_recorder", "rosbag recorder", None, QtGui.QApplication.UnicodeUTF8))
        self.label_3.setText(QtGui.QApplication.translate("rosbag_recorder", "save root dir: ", None, QtGui.QApplication.UnicodeUTF8))
        self.choose_button.setText(QtGui.QApplication.translate("rosbag_recorder", "Choose", None, QtGui.QApplication.UnicodeUTF8))
        self.groupBox.setTitle(QtGui.QApplication.translate("rosbag_recorder", "Information", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("rosbag_recorder", "description:", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("rosbag_recorder", "title:", None, QtGui.QApplication.UnicodeUTF8))
        self.groupBox_2.setTitle(QtGui.QApplication.translate("rosbag_recorder", "command option", None, QtGui.QApplication.UnicodeUTF8))
        self.stop_button.setText(QtGui.QApplication.translate("rosbag_recorder", "Stop", None, QtGui.QApplication.UnicodeUTF8))
        self.start_button.setText(QtGui.QApplication.translate("rosbag_recorder", "Start", None, QtGui.QApplication.UnicodeUTF8))

