# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'main.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(708, 346)
        self.takeoff_land = QtWidgets.QSlider(Form)
        self.takeoff_land.setGeometry(QtCore.QRect(410, 270, 160, 29))
        self.takeoff_land.setStyleSheet("\n"
"    background: qlineargradient(x1: 0, y1: 0.2, x2: 1, y2: 1, stop: 0 #1a7adc, stop: 1 #57a6f4);\n"
"    border: 1px solid #202020;\n"
"    width: 8px;\n"
"    border-radius: 8px;\n"
"")
        self.takeoff_land.setMaximum(1)
        self.takeoff_land.setPageStep(1)
        self.takeoff_land.setOrientation(QtCore.Qt.Horizontal)
        self.takeoff_land.setObjectName("takeoff_land")
        self.Stop = QtWidgets.QPushButton(Form)
        self.Stop.setGeometry(QtCore.QRect(160, 220, 71, 31))
        self.Stop.setObjectName("Stop")
        self.verticalLayoutWidget = QtWidgets.QWidget(Form)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(530, 100, 151, 71))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.trayectorias = QtWidgets.QComboBox(self.verticalLayoutWidget)
        self.trayectorias.setMaxCount(2147483646)
        self.trayectorias.setMinimumContentsLength(0)
        self.trayectorias.setObjectName("trayectorias")
        self.trayectorias.addItem("")
        self.trayectorias.addItem("")
        self.trayectorias.addItem("")
        self.trayectorias.addItem("")
        self.trayectorias.addItem("")
        self.verticalLayout.addWidget(self.trayectorias)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        self.Enviar = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.Enviar.setObjectName("Enviar")
        self.horizontalLayout.addWidget(self.Enviar)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.Emergencia = QtWidgets.QPushButton(Form)
        self.Emergencia.setGeometry(QtCore.QRect(140, 260, 111, 41))
        self.Emergencia.setObjectName("Emergencia")
        self.horizontalLayoutWidget_2 = QtWidgets.QWidget(Form)
        self.horizontalLayoutWidget_2.setGeometry(QtCore.QRect(520, 10, 151, 41))
        self.horizontalLayoutWidget_2.setObjectName("horizontalLayoutWidget_2")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_2)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label = QtWidgets.QLabel(self.horizontalLayoutWidget_2)
        self.label.setObjectName("label")
        self.horizontalLayout_3.addWidget(self.label)
        self.Bateria = QtWidgets.QProgressBar(self.horizontalLayoutWidget_2)
        self.Bateria.setProperty("value", 15)
        self.Bateria.setObjectName("Bateria")
        self.horizontalLayout_3.addWidget(self.Bateria)
        self.horizontalLayoutWidget = QtWidgets.QWidget(Form)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(90, 70, 221, 141))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.gridLayout_2 = QtWidgets.QGridLayout()
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.Sube = QtWidgets.QToolButton(self.horizontalLayoutWidget)
        self.Sube.setArrowType(QtCore.Qt.UpArrow)
        self.Sube.setObjectName("Sube")
        self.gridLayout_2.addWidget(self.Sube, 0, 1, 1, 1)
        self.GiraIzq = QtWidgets.QToolButton(self.horizontalLayoutWidget)
        self.GiraIzq.setArrowType(QtCore.Qt.LeftArrow)
        self.GiraIzq.setObjectName("GiraIzq")
        self.gridLayout_2.addWidget(self.GiraIzq, 2, 0, 1, 1)
        self.Baja = QtWidgets.QToolButton(self.horizontalLayoutWidget)
        self.Baja.setArrowType(QtCore.Qt.DownArrow)
        self.Baja.setObjectName("Baja")
        self.gridLayout_2.addWidget(self.Baja, 3, 1, 1, 1)
        self.GiraDer = QtWidgets.QToolButton(self.horizontalLayoutWidget)
        self.GiraDer.setArrowType(QtCore.Qt.RightArrow)
        self.GiraDer.setObjectName("GiraDer")
        self.gridLayout_2.addWidget(self.GiraDer, 2, 2, 1, 1)
        self.horizontalLayout_2.addLayout(self.gridLayout_2)
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem1)
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.Izquierda = QtWidgets.QToolButton(self.horizontalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Izquierda.sizePolicy().hasHeightForWidth())
        self.Izquierda.setSizePolicy(sizePolicy)
        self.Izquierda.setArrowType(QtCore.Qt.LeftArrow)
        self.Izquierda.setObjectName("Izquierda")
        self.gridLayout.addWidget(self.Izquierda, 1, 0, 1, 1)
        self.Derecha = QtWidgets.QToolButton(self.horizontalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Derecha.sizePolicy().hasHeightForWidth())
        self.Derecha.setSizePolicy(sizePolicy)
        self.Derecha.setArrowType(QtCore.Qt.RightArrow)
        self.Derecha.setObjectName("Derecha")
        self.gridLayout.addWidget(self.Derecha, 1, 2, 1, 1)
        self.Retrocede = QtWidgets.QToolButton(self.horizontalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Retrocede.sizePolicy().hasHeightForWidth())
        self.Retrocede.setSizePolicy(sizePolicy)
        self.Retrocede.setArrowType(QtCore.Qt.DownArrow)
        self.Retrocede.setObjectName("Retrocede")
        self.gridLayout.addWidget(self.Retrocede, 2, 1, 1, 1)
        self.Avanza = QtWidgets.QToolButton(self.horizontalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Avanza.sizePolicy().hasHeightForWidth())
        self.Avanza.setSizePolicy(sizePolicy)
        self.Avanza.setMaximumSize(QtCore.QSize(22, 16777215))
        self.Avanza.setArrowType(QtCore.Qt.UpArrow)
        self.Avanza.setObjectName("Avanza")
        self.gridLayout.addWidget(self.Avanza, 0, 1, 1, 1)
        self.horizontalLayout_2.addLayout(self.gridLayout)
        self.label_2 = QtWidgets.QLabel(Form)
        self.label_2.setGeometry(QtCore.QRect(450, 240, 101, 17))
        self.label_2.setObjectName("label_2")

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.Stop.setText(_translate("Form", "STOP"))
        self.trayectorias.setItemText(0, _translate("Form", "Circular"))
        self.trayectorias.setItemText(1, _translate("Form", "Cuadrada"))
        self.trayectorias.setItemText(2, _translate("Form", "Linea recta"))
        self.trayectorias.setItemText(3, _translate("Form", "Triangular"))
        self.trayectorias.setItemText(4, _translate("Form", "Hexagonal"))
        self.Enviar.setText(_translate("Form", "Enviar"))
        self.Emergencia.setText(_translate("Form", "Emergencia"))
        self.label.setText(_translate("Form", "Batería"))
        self.Sube.setText(_translate("Form", "..."))
        self.GiraIzq.setText(_translate("Form", "..."))
        self.Baja.setText(_translate("Form", "..."))
        self.GiraDer.setText(_translate("Form", "..."))
        self.Izquierda.setText(_translate("Form", "..."))
        self.Derecha.setText(_translate("Form", "..."))
        self.Retrocede.setText(_translate("Form", "..."))
        self.Avanza.setText(_translate("Form", "..."))
        self.label_2.setText(_translate("Form", "Land/Takeoff"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())

