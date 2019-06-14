#Qt5
from PyQt5.QtWidgets import (QWidget, QSlider, QGridLayout, QLayout, QPushButton, QStackedWidget,
    QLabel, QComboBox, QApplication, QTabWidget, QVBoxLayout, QLineEdit)
from PyQt5.Qt3DExtras import Qt3DWindow
from PyQt5 import QtCore, Qt, QtGui
from PyQt5.QtGui import QPixmap, QIcon

#system
import sys
import os
import time
import threading
import shutil
import math # for math.nan
import importlib # dynamic imports
import re # urdf editing through regex
import zipfile # zip (de)compression for 3D models

from billiard import Process

# component rviz visualization
import fastLaunch

# ROS2
import rclpy
from hrim_generic_msgs.msg import Power
from hrim_generic_srvs.srv import ID, Simulation3D
from rclpy.executors import SingleThreadedExecutor as localExecutor

from rclpy.qos import qos_profile_sensor_data as QOS

IMPORT_TYPES = {
    "Gripper":{
        "specs":{"package": "hrim_actuator_gripper_srvs.srv",
                 "class": "SpecsFingerGripper",
                 "topic": "specs"
                },
        "state":{"package": "hrim_actuator_gripper_msgs.msg",
                 "class": "StateFingerGripper",
                 "topic": ["fingerstate"],
                 "property":{
                             "Position": "linear_position",
                             "Rotation": "angular_position"
                            }
                },
        "control":{"package": "hrim_actuator_gripper_srvs.srv",
                   "class": "ControlFinger",
                   "type": "service",
                   "topic": ["fingercontrol"]
                }
    },
    "Servo":{
        "specs":{"package": "hrim_actuator_rotaryservo_srvs.srv",
                 "class": "SpecsRotaryServo",
                 "topic": "specs"
                },
        "state":{"package": "hrim_actuator_rotaryservo_msgs.msg",
                 "class": "StateRotaryServo",
                 "topic": ["state_axis1", "state_axis2"],
                 "property":{
                             "Position": "position"
                            }
                },
        "control":{"package": "hrim_actuator_rotaryservo_msgs.msg",
                   "class": "GoalRotaryServo",
                   "type": "topic",
                   "topic": ["goal_axis1", "goal_axis2"]
                  }
    }
}

class ComboBox(QComboBox):
    popupAboutToBeShown = QtCore.pyqtSignal()

    def showPopup(self):
        self.popupAboutToBeShown.emit()
        super(ComboBox, self).showPopup()

class HrimGripperControl(QWidget):

    # Class initializer
    def __init__(self, app):
        try:
            super().__init__()

            self.app = app

            self.context = rclpy.context.Context()
            rclpy.init(context=self.context)

            self.node = rclpy.create_node('hrim_control_'+str(threading.get_ident()), context=self.context)

            self.executor = localExecutor(context=self.context)
            self.executor.add_node(self.node)

            # Flag for thread termination
            self.keepThreads = True

            # Whether currently selected node is a valid HRIM node
            self.validNode = False

            # Currently selected node name
            self.node_name = None

            self.classSpecs = None
            self.classState = None
            self.classControl = None

            # Amount of tries for state callback
            self.maxStateCounter = 15
            self.stateCounter = 0


            # Gripper goal linear position value
            self.firstValue = 0.0
            self.secondValue = 0.0

            # Gripper goal linear position maximum value
            self.maxPosition = None

            self.withAction = False

            # Joint rated speed
            self.rated_speed = 1

            # Component compressed 3D model meshes+urdf
            self.model = None
            # Name of the model's base frame needed for visualization
            self.model_frame = None

            self.streaming = False

            self.state_subs = []

            self.streaming_pubs = []
            self.streaming_msgs = []

            self.lastState = None

            self.initUI()

            watcher = threading.Thread(target=self.workerWatcher)
            watcher.start()

            self.show()

        except KeyboardInterrupt:
            self.closeEvent()
            raise
        except:
            raise

    # UI initialization
    def initUI(self):

        # self.setFixedSize(430,300)
        self.setFixedWidth(460)


        vBox = QVBoxLayout()
        vBox.setAlignment(QtCore.Qt.AlignTop)
        # resize to minimal space on content changes
        vBox.setSizeConstraint(QLayout.SetFixedSize)

        self.setLayout(vBox)


        ### Node selection

        # Thread count label
        self.wLblNodeCount = QLabel("Node")

        # Node select combobox
        self.wSelNode = ComboBox()
        self.wSelNode.addItem("")
        self.wSelNode.activated[int].connect(self.nodeSelected)
        self.wSelNode.popupAboutToBeShown.connect(self.listNodes)

        # Disconect button
        wBtnRefresh = QPushButton("")
        wBtnRefresh.setIcon(QIcon('img/refresh.png'))
        wBtnRefresh.clicked.connect(self.clickedOnRefresh)

        # Node selection widget
        laySelect = QWidget()
        laySelect.layout = QGridLayout()
        laySelect.layout.addWidget(self.wLblNodeCount, 0, 0)
        laySelect.layout.addWidget(self.wSelNode, 0, 1, 1, 10)
        laySelect.layout.addWidget(wBtnRefresh, 0, 11)
        laySelect.setLayout(laySelect.layout)
        laySelect.setFixedSize(400,50)

        vBox.addWidget(laySelect)

        ### Control

        regex=QtCore.QRegExp("^-?[0-9]*[.]?[0-9]*$")
        validator = QtGui.QRegExpValidator(regex)

        ## Gripper

        # Gripper position
        self.wBtnGripperPos = QPushButton("Position")
        self.wBtnGripperPos.setCheckable(True)
        self.wBtnGripperPos.setEnabled(False)
        self.wBtnGripperPos.clicked[bool].connect(self.wBtnGripperPosToggle)

        self.wInGripperPos = QLineEdit()
        self.wInGripperPos.setFixedWidth(60)

        self.wInGripperPos.setValidator(validator)
        self.wInGripperPos.setText("0.000")
        self.wInGripperPos.returnPressed.connect(self.wInGripperPosChange)
        self.wInGripperPos.setEnabled(False)

        self.wSldGripperPos = QSlider(QtCore.Qt.Horizontal, self)
        self.wSldGripperPos.setMinimum(0)
        self.wSldGripperPos.setMaximum(1000)
        self.wSldGripperPos.setFocusPolicy(QtCore.Qt.NoFocus)
        self.wSldGripperPos.setGeometry(30, 40, 100, 30)
        self.wSldGripperPos.valueChanged[int].connect(self.wSldGripperPosChange)
        self.wSldGripperPos.setEnabled(False)

        self.wLblGripperPosMax = QLabel("0")
        self.wLblGripperPosMax.setEnabled(False)

        # Gripper rotation
        self.wBtnGripperRot = QPushButton("Rotation")
        self.wBtnGripperRot.setCheckable(True)
        self.wBtnGripperRot.setEnabled(False)
        self.wBtnGripperRot.clicked[bool].connect(self.wBtnGripperRotToggle)

        self.wInGripperRot = QLineEdit()
        self.wInGripperRot.setFixedWidth(60)

        self.wInGripperRot.setValidator(validator)
        self.wInGripperRot.setText("0.000")
        self.wInGripperRot.returnPressed.connect(self.wInGripperRotChange)
        self.wInGripperRot.setEnabled(False)

        self.wSldGripperRot = QSlider(QtCore.Qt.Horizontal, self)
        self.wSldGripperRot.setMinimum(0)
        self.wSldGripperRot.setMaximum(1000)
        self.wSldGripperRot.setFocusPolicy(QtCore.Qt.NoFocus)
        self.wSldGripperRot.setGeometry(30, 40, 100, 30)
        self.wSldGripperRot.valueChanged[int].connect(self.wSldGripperRotChange)
        self.wSldGripperRot.setEnabled(False)

        self.wLblGripperRotMax = QLabel("0")
        self.wLblGripperRotMax.setEnabled(False)

        # Gripper control stack
        self.wStackGripper = QWidget()
        self.wStackGripper.layout = QGridLayout()
        self.wStackGripper.layout.setAlignment(QtCore.Qt.AlignTop)

        self.wStackGripper.layout.addWidget(self.wBtnGripperPos, 0, 0, 1, 2)
        self.wStackGripper.layout.addWidget(self.wInGripperPos, 0, 2)
        self.wStackGripper.layout.addWidget(self.wSldGripperPos, 0, 3, 1, 6)
        self.wStackGripper.layout.addWidget(self.wLblGripperPosMax, 0, 9)

        self.wStackGripper.layout.addWidget(self.wBtnGripperRot, 1, 0, 1, 2)
        self.wStackGripper.layout.addWidget(self.wInGripperRot, 1, 2)
        self.wStackGripper.layout.addWidget(self.wSldGripperRot, 1, 3, 1, 6)
        self.wStackGripper.layout.addWidget(self.wLblGripperRotMax, 1, 9)

        # self.wStackGripper.setStyleSheet("background-color:black;")
        self.wStackGripper.setLayout(self.wStackGripper.layout)
        self.wStackGripper.setFixedSize(400,100)

        ## Joint

        # Joint axis 1
        self.wBtnJointAxis1 = QPushButton("Axis 1")
        self.wBtnJointAxis1.setCheckable(True)
        self.wBtnJointAxis1.setEnabled(False)
        self.wBtnJointAxis1.clicked[bool].connect(self.wBtnJointAxis1Toggle)

        self.wInJointAxis1 = QLineEdit()
        self.wInJointAxis1.setFixedWidth(60)

        self.wInJointAxis1.setValidator(validator)
        self.wInJointAxis1.setText("0.000")
        self.wInJointAxis1.returnPressed.connect(self.wInJointAxis1Change)
        self.wInJointAxis1.setEnabled(False)

        self.wSldJointAxis1 = QSlider(QtCore.Qt.Horizontal, self)
        self.wSldJointAxis1.setMinimum(0)
        self.wSldJointAxis1.setMaximum(1000)
        self.wSldJointAxis1.setValue(500)
        self.wSldJointAxis1.setFocusPolicy(QtCore.Qt.NoFocus)
        self.wSldJointAxis1.setGeometry(30, 40, 100, 30)
        self.wSldJointAxis1.valueChanged[int].connect(self.wSldJointAxis1Change)
        self.wSldJointAxis1.setEnabled(False)

        self.wLblJointAxis1Max = QLabel("0")
        self.wLblJointAxis1Max.setEnabled(False)

        # Joint rotation
        self.wBtnJointAxis2 = QPushButton("Axis 2")
        self.wBtnJointAxis2.setCheckable(True)
        self.wBtnJointAxis2.setEnabled(False)
        self.wBtnJointAxis2.clicked[bool].connect(self.wBtnJointAxis2Toggle)

        self.wInJointAxis2 = QLineEdit()
        self.wInJointAxis2.setFixedWidth(60)

        self.wInJointAxis2.setValidator(validator)
        self.wInJointAxis2.setText("0.000")
        self.wInJointAxis2.returnPressed.connect(self.wInJointAxis2Change)
        self.wInJointAxis2.setEnabled(False)

        self.wSldJointAxis2 = QSlider(QtCore.Qt.Horizontal, self)
        self.wSldJointAxis2.setMinimum(0)
        self.wSldJointAxis2.setMaximum(1000)
        self.wSldJointAxis2.setValue(500)
        self.wSldJointAxis2.setFocusPolicy(QtCore.Qt.NoFocus)
        self.wSldJointAxis2.setGeometry(30, 40, 100, 30)
        self.wSldJointAxis2.valueChanged[int].connect(self.wSldJointAxis2Change)
        self.wSldJointAxis2.setEnabled(False)

        self.wLblJointAxis2Max = QLabel("0")
        self.wLblJointAxis2Max.setEnabled(False)

        # Joint goal streaming button
        self.wBtnJointStreaming = QPushButton("Stream goal")
        self.wBtnJointStreaming.setCheckable(True)
        self.wBtnJointStreaming.setEnabled(False)
        self.wBtnJointStreaming.clicked[bool].connect(self.wBtnJointStreamingToggle)

        # Joint control stack
        self.wStackJoint = QWidget()
        self.wStackJoint.layout = QGridLayout()
        self.wStackJoint.layout.setAlignment(QtCore.Qt.AlignTop)

        self.wStackJoint.layout.addWidget(self.wBtnJointAxis1, 0, 0, 1, 2)
        self.wStackJoint.layout.addWidget(self.wInJointAxis1, 0, 2)
        self.wStackJoint.layout.addWidget(self.wSldJointAxis1, 0, 3, 1, 6)
        self.wStackJoint.layout.addWidget(self.wLblJointAxis1Max, 0, 9)

        self.wStackJoint.layout.addWidget(self.wBtnJointAxis2, 1, 0, 1, 2)
        self.wStackJoint.layout.addWidget(self.wInJointAxis2, 1, 2)
        self.wStackJoint.layout.addWidget(self.wSldJointAxis2, 1, 3, 1, 6)
        self.wStackJoint.layout.addWidget(self.wLblJointAxis2Max, 1, 9)

        self.wStackJoint.layout.addWidget(self.wBtnJointStreaming, 2, 0, 1, 10)

        self.wStackJoint.setLayout(self.wStackJoint.layout)
        self.wStackJoint.setFixedSize(400,100)

        ## Control input section
        self.layControlInput = QStackedWidget (self)
        self.layControlInput.addWidget(self.wStackGripper)
        self.layControlInput.addWidget(self.wStackJoint)
        # self.layControlInput.setFixedSize(400,300)

        ## Send order button
        self.wBtnGo = QPushButton("Send order")
        self.wBtnGo.clicked.connect(self.wBtnGoClicked)
        self.wBtnGo.setEnabled(False)

        ## Overall control section
        self.layControl = QWidget()
        self.layControl.layout = QGridLayout()
        self.layControl.layout.addWidget(self.layControlInput)
        self.layControl.layout.addWidget(self.wBtnGo)
        self.layControl.setLayout(self.layControl.layout)
        self.layControl.hide()

        vBox.addWidget(self.layControl)

        ### Component information tabs
        self.layTabs = QTabWidget()

        ## ID tab
        wTabId = QWidget()
        wTabId.layout = QGridLayout()

        wLblIdCategory = QLabel("Device kind:")
        self.wLblIdCategoryValue = QLabel("")
        wLblIdType = QLabel("Device name:")
        self.wLblIdTypeValue = QLabel("")
        wLblIdHrim = QLabel("HRIM version:")
        self.wLblIdHrimValue = QLabel("")
        wLblIdHros = QLabel("HROS version:")
        self.wLblIdHrosValue = QLabel("")

        wTabId.layout.addWidget(wLblIdCategory, 0, 0)
        wTabId.layout.addWidget(self.wLblIdCategoryValue, 0, 1)
        wTabId.layout.addWidget(wLblIdType, 1, 0)
        wTabId.layout.addWidget(self.wLblIdTypeValue, 1, 1)
        wTabId.layout.addWidget(wLblIdHrim, 2, 0)
        wTabId.layout.addWidget(self.wLblIdHrimValue, 2, 1)
        wTabId.layout.addWidget(wLblIdHros, 3, 0)
        wTabId.layout.addWidget(self.wLblIdHrosValue, 3, 1)

        wTabId.setLayout(wTabId.layout)

        ## State tab
        self.wTabState = QWidget()
        self.wTabState.layout = QGridLayout()

        self.wLblStateFirst = QLabel("")
        self.wLblStateFirstValue = QLabel("0.000")
        self.wLblStateSecond = QLabel("")
        self.wLblStateSecondValue = QLabel("0.000")

        self.wTabState.layout.addWidget(self.wLblStateFirst, 0, 0)
        self.wTabState.layout.addWidget(self.wLblStateFirstValue, 0, 1)
        self.wTabState.layout.addWidget(self.wLblStateSecond, 1, 0)
        self.wTabState.layout.addWidget(self.wLblStateSecondValue, 1, 1)

        self.wTabState.setLayout(self.wTabState.layout)

        ## Power tab
        wTabPower = QWidget()
        wTabPower.layout = QGridLayout()

        wLblPowerPowerConsumption = QLabel("Power consumption:")
        self.wLblPowerPowerConsumptionValue = QLabel("0.000")
        wLblPowerCurrentConsumption = QLabel("Current consumption:")
        self.wLblPowerCurrentConsumptionValue = QLabel("0.00000")
        wLblPowerVoltage = QLabel("Voltage:")
        self.wLblPowerVoltageValue = QLabel("0.000")

        wTabPower.layout.addWidget(wLblPowerVoltage, 0, 0)
        wTabPower.layout.addWidget(self.wLblPowerVoltageValue, 0, 1)
        wTabPower.layout.addWidget(wLblPowerPowerConsumption, 1, 0)
        wTabPower.layout.addWidget(self.wLblPowerPowerConsumptionValue, 1, 1)
        wTabPower.layout.addWidget(wLblPowerCurrentConsumption, 2, 0)
        wTabPower.layout.addWidget(self.wLblPowerCurrentConsumptionValue, 2, 1)

        wTabPower.setLayout(wTabPower.layout)


        wTab3D = QWidget()
        wTab3D.layout = QGridLayout()

        wBtn3DRviz = QPushButton("rviz2")
        wBtn3DRviz.clicked.connect(self.launchRviz)

        wTab3D.layout.addWidget(wBtn3DRviz, 0,0)

        wTab3D.setLayout(wTab3D.layout)


        ## Add tabs
        self.layTabs.addTab(wTabId,"ID")
        self.layTabs.addTab(self.wTabState,"State")
        self.layTabs.addTab(wTabPower,"Power")
        self.layTabs.addTab(wTab3D,"3D")
        self.layTabs.hide()

        vBox.addWidget(self.layTabs)

    # Closing event override
    def closeEvent(self, event=None):
        try:

            # Signal worker threads should stop
            self.keepThreads = False

            # Give time to threads to stop
            time.sleep(0.1)

            if os.path.exists(os.path.join(os.getcwd(),"tmp")):
                shutil.rmtree(os.path.join(os.getcwd(),"tmp"))

            # ROS2 cleanup
            self.executor.shutdown()
            self.node.destroy_node()
            rclpy.shutdown(context=self.context)

            sys.exit(0)
        except:
            raise

    #########################################################
    # UI interaction handlers                               #
    #########################################################

    def nodeSelected(self, value):
        try:

            self.validNode = False
            counter = 0
            while(threading.active_count()>2 and counter<100):
                time.sleep(0.01)

            self.resetAll()

            if value>0 and counter<100:
                self.node_name = self.node_list[value-1]
                if "/"+self.node_name+"/specs" in dict(self.node.get_service_names_and_types()):
                    self.validNode = True
                    self.connectToNode()
                else:
                    self.validNode = False
                    print("Not a valid HRIM gripper node")
        except:
            raise

    def clickedOnRefresh(self):
        try:
            self.validNode = False

            self.wSelNode.blockSignals(True)
            self.wSelNode.setCurrentIndex(0)
            self.wSelNode.blockSignals(False)

            self.cleanup()

            # self.resetUI()
            self.resetAll()

        except:
            raise

    def wBtnGoClicked(self):
        try:
            if self.type == "Gripper":
                arg1 = self.firstValue if self.wSldGripperPos.isEnabled() else math.nan
                arg2 = self.secondValue if self.wSldGripperRot.isEnabled() else math.nan
            if self.type == "Servo":
                arg1 = self.firstValue if self.wSldJointAxis1.isEnabled() else math.nan
                arg2 = self.secondValue if self.wSldJointAxis2.isEnabled() else math.nan

            t = threading.Thread(target=self.workerControl, args=[arg1, arg2])
            t.start()
        except:
            raise

    ## Gripper
    def wInGripperPosChange(self):
        try:
            value = self.wInGripperPos.text()
            if len(value)<1 or value in [".", "-"] or float(value)==0.0:
                val = 0.0
                self.wSldGripperPos.setValue(0)
            else:
                val = float(value)
                self.wSldGripperPos.setValue((val*1000)/self.maxPosition)
            self.firstValue = val
        except:
            raise

    def wSldGripperPosChange(self, value):
        try:
            self.firstValue = float((self.maxPosition/1000)*value)
            self.wInGripperPos.setText("%.3f" % self.firstValue)
            self.wInGripperPos.update()
        except:
            raise

    def wInGripperRotChange(self):
        try:
            value = self.wInGripperRot.text()
            if len(value)<1 or value in [".", "-"] or float(value)==0.0:
                val = 0.0
                self.wSldGripperRot.setValue(0)
            else:
                val = float(value)
                self.wSldGripperRot.setValue((val*1000)/self.maxRotation)
            self.secondValue = val
        except:
            raise

    def wSldGripperRotChange(self, value):
        try:
            self.secondValue = float((self.maxRotation/1000)*value)
            self.wInGripperRot.setText("%.3f" % self.secondValue)
            self.wInGripperRot.update()
        except:
            raise

    def wBtnGripperPosToggle(self, isChecked):
        try:
            self.wInGripperPos.setEnabled(isChecked)
            self.wLblGripperPosMax.setEnabled(isChecked)
            self.wSldGripperPos.setEnabled(isChecked)

            self.wBtnGo.setEnabled(any([isChecked, self.wBtnGripperRot.isChecked()]))
        except:
            raise

    def wBtnGripperRotToggle(self, isChecked):
        try:
            self.wInGripperRot.setEnabled(isChecked)
            self.wLblGripperRotMax.setEnabled(isChecked)
            self.wSldGripperRot.setEnabled(isChecked)

            self.wBtnGo.setEnabled(any([isChecked, self.wBtnGripperPos.isChecked()]))
        except:
            raise

    ## Joints
    def wInJointAxis1Change(self):
        try:
            value = self.wInJointAxis1.text()
            if len(value)<1 or value in [".", "-"] or float(value)==0.0:
                val = 0.0
                self.wSldJointAxis1.blockSignals(True)
                self.wSldJointAxis1.setValue(500)
                self.wSldJointAxis1.blockSignals(False)
            else:
                val = float(value)
                self.wSldJointAxis1.blockSignals(True)
                self.wSldJointAxis1.setValue(1000/((self.maxJoint-self.minJoint)/(val-self.minJoint)))
                self.wSldJointAxis1.blockSignals(False)
            self.firstValue = val


        except:
            raise

    def wSldJointAxis1Change(self, value):
        try:
            range = self.maxJoint - self.minJoint
            self.firstValue = float((range/1000)*value)+self.minJoint

            self.wInJointAxis1.blockSignals(True)
            self.wInJointAxis1.setText("%.3f" % self.firstValue)
            self.wInJointAxis1.update()
            self.wInJointAxis1.blockSignals(False)

        except:
            raise

    def wInJointAxis2Change(self):
        try:
            value = self.wInJointAxis2.text()
            if len(value)<1 or value in [".", "-"] or float(value)==0.0:
                val = 0.0

                self.wSldJointAxis2.blockSignals(True)
                self.wSldJointAxis2.setValue(500)
                self.wSldJointAxis2.blockSignals(False)
            else:
                val = float(value)
                self.wSldJointAxis2.blockSignals(True)
                self.wSldJointAxis2.setValue(1000/((self.maxJoint-self.minJoint)/(val-self.minJoint)))
                self.wSldJointAxis2.blockSignals(False)
            self.secondValue = val

        except:
            raise

    def wSldJointAxis2Change(self, value):
        try:
            range = self.maxJoint - self.minJoint
            self.secondValue = float((range/1000)*value)+self.minJoint

            self.wInJointAxis2.blockSignals(True)
            self.wInJointAxis2.setText("%.3f" % self.secondValue)
            self.wInJointAxis2.update()
            self.wInJointAxis2.blockSignals(False)

        except:
            raise

    def wBtnJointAxis1Toggle(self, isChecked):
        try:
            self.wInJointAxis1.setEnabled(isChecked)
            self.wLblJointAxis1Max.setEnabled(isChecked)
            self.wSldJointAxis1.setEnabled(isChecked)

            self.wBtnGo.setEnabled(any([isChecked, self.wBtnJointAxis2.isChecked()]))
            self.wBtnJointStreaming.setEnabled(any([isChecked, self.wBtnJointAxis2.isChecked()]))
            if not any([isChecked, self.wBtnJointAxis2.isChecked()]):
                self.wBtnJointStreaming.setChecked(False)
        except:
            raise

    def wBtnJointAxis2Toggle(self, isChecked):
        try:
            self.wInJointAxis2.setEnabled(isChecked)
            self.wLblJointAxis2Max.setEnabled(isChecked)
            self.wSldJointAxis2.setEnabled(isChecked)

            self.wBtnGo.setEnabled(any([isChecked, self.wBtnJointAxis1.isChecked()]))
            self.wBtnJointStreaming.setEnabled(any([isChecked, self.wBtnJointAxis1.isChecked()]))
            if not any([isChecked, self.wBtnJointAxis1.isChecked()]):
                self.wBtnJointStreaming.setChecked(False)
        except:
            raise

    def wBtnJointStreamingToggle(self, isChecked):
        try:
            self.streaming = isChecked
            if isChecked:
                threadStreaming = threading.Thread(target=self.workerStreaming)
                threadStreaming.start()
        except:
            raise

    #########################################################
    # UI manipulation                                       #
    #########################################################

    def resetAll(self):

        # for x in range(0, len(self.state_subs)):
        #     self.node.destroy_subscription(self.state_subs[x])

        self.state_subs = []
        self.streaming_pubs = []
        self.streaming_msgs = []

        try:
            self.resetUI()
            self.resetCounters()
            # self.resetState()
            # self.resetID()
        except:
            raise

    def resetUI(self):
        try:
            self.resetControl()
            self.layTabs.setCurrentIndex(0)
            self.layTabs.setTabEnabled(2, False);
            self.layTabs.setTabEnabled(3, False);
        except:
            raise

    def resetControl(self):
        try:

            self.wBtnGripperPos.blockSignals(True)
            self.wInGripperPos.blockSignals(True)
            self.wSldGripperPos.blockSignals(True)

            self.wBtnGripperPos.setChecked(False)
            self.wBtnGripperPosToggle(False)
            self.wInGripperPos.setText("0.000")
            self.wInGripperPos.setEnabled(False)
            self.wLblGripperPosMax.setText("0.000")
            self.wLblGripperPosMax.setEnabled(False)
            self.wSldGripperPos.setValue(0)
            self.wSldGripperPos.setEnabled(False)

            self.wBtnGripperPos.blockSignals(False)
            self.wInGripperPos.blockSignals(False)
            self.wSldGripperPos.blockSignals(False)

            self.wBtnGripperRot.blockSignals(True)
            self.wInGripperRot.blockSignals(True)
            self.wSldGripperRot.blockSignals(True)

            self.wBtnGripperRot.setChecked(False)
            self.wBtnGripperRotToggle(False)
            self.wInGripperRot.setText("0.000")
            self.wInGripperRot.setEnabled(False)
            self.wLblGripperRotMax.setText("0.000")
            self.wLblGripperRotMax.setEnabled(False)
            self.wSldGripperRot.setValue(0)
            self.wSldGripperRot.setEnabled(False)

            self.wBtnGripperRot.blockSignals(False)
            self.wInGripperRot.blockSignals(False)
            self.wSldGripperRot.blockSignals(False)

            self.wBtnJointAxis1.blockSignals(True)
            self.wInJointAxis1.blockSignals(True)
            self.wSldJointAxis1.blockSignals(True)

            self.wBtnJointAxis1.setChecked(False)
            self.wBtnJointAxis1Toggle(False)
            self.wInJointAxis1.setText("0.000")
            self.wInJointAxis1.setEnabled(False)
            self.wLblJointAxis1Max.setText("0.000")
            self.wLblJointAxis1Max.setEnabled(False)
            self.wSldJointAxis1.setValue(500)
            self.wSldJointAxis1.setEnabled(False)

            self.wBtnJointAxis1.blockSignals(False)
            self.wInJointAxis1.blockSignals(False)
            self.wSldJointAxis1.blockSignals(False)

            self.wBtnJointAxis2.blockSignals(True)
            self.wInJointAxis2.blockSignals(True)
            self.wSldJointAxis2.blockSignals(True)

            self.wBtnJointAxis2.setChecked(False)
            self.wBtnJointAxis2Toggle(False)
            self.wInJointAxis2.setText("0.000")
            self.wInJointAxis2.setEnabled(False)
            self.wLblJointAxis2Max.setText("0.000")
            self.wLblJointAxis2Max.setEnabled(False)
            self.wSldJointAxis2.setValue(500)
            self.wSldJointAxis2.setEnabled(False)

            self.wBtnJointAxis2.blockSignals(False)
            self.wInJointAxis2.blockSignals(False)
            self.wSldJointAxis2.blockSignals(False)

            self.wBtnJointStreaming.blockSignals(True)

            self.wBtnJointStreaming.setChecked(False)
            self.wBtnJointStreaming.setEnabled(False)

            self.wBtnJointStreaming.blockSignals(False)

            self.wBtnGo.setEnabled(False)


            self.wLblStateFirstValue.setText("0.000")
            self.wLblStateSecondValue.setText("0.000")


            self.layControl.hide()
            self.layTabs.hide()

            self.wSelNode.blockSignals(False)

            # self.listNodes()

        except:
            raise

    def resetCounters(self):
        try:
            self.stateCounter = 0
            self.firstValue = 0.0
            self.secondValue = 0.0
        except:
            raise

    def handleId(self, result):
        hrim_types = ["Communication", "Sensor", "Actuator", "Cognition", "UI", "Power", "Composite"]
        self.wLblIdCategoryValue.setText(hrim_types[result.device_kind_id])
        self.wLblIdTypeValue.setText(result.device_name)
        self.wLblIdHrimValue.setText(result.hrim_version)
        self.wLblIdHrosValue.setText(result.hros_version)

    def handleSpecs(self, result):

        if self.type == "Gripper":
            self.maxPosition = float("%.3f" % result.max_length)
            self.wLblGripperPosMax.setText(str(self.maxPosition))
            self.wBtnGripperPos.setEnabled(True)

            self.wLblStateFirst.setText("Position:")
            self.wLblStateSecond.setText("Rotation:")

            if(result.max_angle > 0):
                print("got angle")
                self.maxRotation = float("%.3f" % result.max_angle)
                self.wLblGripperRotMax.setText(str(self.maxRotation))
                self.wBtnGripperRot.setEnabled(True)

            self.layControlInput.setCurrentIndex(0)

        elif self.type == "Servo":
            self.rated_speed = result.rated_speed
            self.minJoint = float("%.3f" % result.range_min)
            self.maxJoint = float("%.3f" % result.range_max)

            self.wLblJointAxis1Max.setText(str(self.maxJoint))
            self.wLblJointAxis2Max.setText(str(self.maxJoint))

            self.wBtnJointAxis1.setEnabled(True)
            self.wBtnJointAxis2.setEnabled(True)

            self.wLblStateFirst.setText("Axis 1:")
            self.wLblStateSecond.setText("Axis 2:")

            self.layControlInput.setCurrentIndex(1)

        self.layControl.show()
        self.layTabs.show()

    def callbackFingerState(self, data):
        try:
            self.stateCounter = 0
            self.wLblStateFirstValue.setText("%.3f" % data.linear_position)
            self.wLblStateSecondValue.setText("%.3f" % data.angular_position)
        except:
            raise

    def callbackServoFirstState(self, data):
        try:
            self.stateCounter = 0
            self.wLblStateFirstValue.setText("%.3f" % data.position)
        except:
            raise

    def callbackServoSecondState(self, data):
        try:
            self.stateCounter = 0
            self.wLblStateSecondValue.setText("%.3f" % data.position)
        except:
            raise

    def callbackPower(self, data):
        try:
            if not self.layTabs.isTabEnabled(2):
                self.layTabs.setTabEnabled(2, True);
            self.wLblPowerVoltageValue.setText("%.3f" % data.voltage)
            self.wLblPowerPowerConsumptionValue.setText("%.3f" % data.power_consumption)
            self.wLblPowerCurrentConsumptionValue.setText("%.5f" % data.current_consumption)
        except:
            raise

    #########################################################
    # Thread workers                                        #
    #########################################################

    def workerWatcher(self):
        try:
            while (self.keepThreads):
                self.wLblNodeCount.setText(str(threading.active_count()-1))
                time.sleep(0.05)
            sys.exit()
        except:
            raise

    def workerSpinner(self):
        try:
            while (self.keepThreads and self.validNode):
                # Any value for the spin_once timeout other than 0 ends un crashing the interface
                self.executor.spin_once(0)
                time.sleep(0.01)
            sys.exit()
        except:
            raise

    # Identification service worker
    def workerID(self):
        try:
            topic_name = "id"
            client = self.node.create_client(
                ID, self.node_name+"/"+topic_name)

            counter = 10
            while self.keepThreads and not client.wait_for_service(timeout_sec=0.5) and counter > 0:
                if(counter%2 == 0):
                    self.node.get_logger().info(topic_name+' service not available, waiting again...')
                counter -= 1

            if counter <=0 or not self.keepThreads:
                self.node.get_logger().info(topic_name+' service not available, please try again...')
                sys.exit()

            future = client.call_async(ID.Request())

            counter = 10
            while(future.result() is None and counter > 0):
                counter -= 1
                time.sleep(0.1)

            if counter <=0:
                self.node.get_logger().info('Call to '+topic_name+' service timed out.')

            if future.result() is not None:
                self.setIdInfo(future.result())
            else:
                self.node.get_logger().info('Service call failed %r' % (future.exception()))

            sys.exit()
        except:
            raise

    # Specs service worker
    def workerSpecs(self):
        try:
            topic_name = "specs"

            client = self.node.create_client(
                self.classSpecs, "/"+self.node_name+"/"+topic_name)

            print("Creating service: ", self.node_name+"/"+topic_name, self.classSpecs, )

            counter = 10

            while self.keepThreads and self.validNode and not client.wait_for_service(timeout_sec=1.0) and counter > 0:
                self.node.get_logger().info('service not available, waiting again...')
                counter -= 1
            # while self.keepThreads and self.validNode and not client.wait_for_service(timeout_sec=0.5) and counter > 0:
            #     if(counter%2 == 0):
            #         self.node.get_logger().info(topic_name+' service not available, waiting again...')
            #     counter -= 1

            if counter <=0 or not self.keepThreads:
                self.node.get_logger().info(topic_name+' service not available, please try again...')
                sys.exit()

            future = client.call_async(self.classSpecs.Request())

            counter = 10
            while(future.result() is None and counter > 0):
                counter -= 1
                time.sleep(0.1)

            if counter <=0:
                self.node.get_logger().info('Call to '+topic_name+' service timed out.')

            if future.result() is not None:
                self.handleSpecs(future.result())

            else:
                self.node.get_logger().info('Service call failed %r' % (future.exception()))

            sys.exit()
        except:
            raise

    def workerSimulation(self):
        try:
            topic_name = "module_3d"

            client = self.node.create_client(
                Simulation3D, self.node_name+"/"+topic_name)

            print("Simulation ", "/"+self.node_name+"/"+topic_name, Simulation3D)

            counter = 10
            while self.keepThreads and self.validNode and not client.wait_for_service(timeout_sec=1.0) and counter > 0:
                self.node.get_logger().info('service not available, waiting again...')
                counter -= 1

            if counter <=0 or not self.keepThreads:
                self.node.get_logger().info(topic_name+' service not available, please try again...')
                # sys.exit()

            future = client.call_async(Simulation3D.Request())

            counter = 10
            while(future.result() is None and counter > 0):
                counter -= 1
                time.sleep(0.1)

            if counter <=0:
                self.node.get_logger().info('Call to '+topic_name+' service timed out.')

            if future.result() is not None:
                self.node.get_logger().info('Service call ok' )

                self.model_frame = future.result().model_name
                self.model = future.result().model
                if len(self.model)>0:
                    self.decompress()
                    self.layTabs.setTabEnabled(3, True);

            else:
                self.node.get_logger().info('Service call failed %r' % (future.exception()))

            sys.exit()
        except:
            raise

    def workerState(self):
        try:
            for x in range(0, len(self.state_subs)):
                self.node.destroy_subscription(self.state_subs[x])

            self.state_subs = []

            if self.type == "Servo":
                print("Servo")

                # for topic in IMPORT_TYPES[self.type]["state"]["topic"]:
                #     self.state_subs.append(self.node.create_subscription(
                #                           self.classState, "/"+self.node_name+"/"+topic,
                #                           self.callbackServoState, qos_profile=QOS
                #                       ))
                self.state_subs.append(self.node.create_subscription(
                                      self.classState, "/"+self.node_name+"/state_axis1",
                                      self.callbackServoFirstState, qos_profile= QOS
                                  ))
                print("/"+self.node_name+"/state_axis1", self.classState)

                self.state_subs.append(self.node.create_subscription(
                                      self.classState, "/"+self.node_name+"/state_axis2",
                                      self.callbackServoSecondState, qos_profile= QOS
                                  ))
                print("/"+self.node_name+"/state_axis2", self.classState)

            if self.type == "Gripper":
                print("Gripper")
                for topic in IMPORT_TYPES[self.type]["state"]["topic"]:
                    print("State: /"+self.node_name+"/"+topic, self.classState)
                    self.state_subs.append(self.node.create_subscription(
                                          self.classState, "/"+self.node_name+"/"+topic,
                                          self.callbackFingerState)
                                         )

            power_sub = self.node.create_subscription(Power, "/"+self.node_name+"/power", self.callbackPower)

            while (self.keepThreads and self.validNode and self.stateCounter<self.maxStateCounter):
                self.stateCounter+=1
                time.sleep(0.1)

            for x in range(0, len(self.state_subs)):
                self.node.destroy_subscription(self.state_subs[x])

            if(self.keepThreads and self.validNode):
                print("State timeout")
                time.sleep(0.05)
                self.clickedOnRefresh()

            sys.exit()
        except:
            raise

    def workerStreaming(self):
        try:
            for x in range(0, len(self.streaming_pubs)):
                self.node.destroy_publisher(self.streaming_pubs[x])

            self.streaming_pubs = []
            self.streaming_msgs = []

            for topic in IMPORT_TYPES[self.type]["control"]["topic"]:
                self.streaming_pubs.append(self.node.create_publisher(self.classControl,
                                          self.node_name+"/"+topic)
                                         )
                msg = self.classControl()
                msg.velocity = self.rated_speed
                self.streaming_msgs.append(msg)

            while (self.streaming and self.keepThreads and self.validNode):
                self.streaming_msgs[0].position = self.firstValue if self.wSldJointAxis1.isEnabled() else math.nan
                self.streaming_msgs[1].position = self.secondValue if self.wSldJointAxis2.isEnabled() else math.nan
                for index in range(0,len(self.streaming_pubs)):
                    self.streaming_pubs[index].publish(self.streaming_msgs[index])
                time.sleep(0.1)

            for x in range(0, len(self.streaming_pubs)):
                self.node.destroy_publisher(self.streaming_pubs[x])


            sys.exit()
        except:
            raise

    def workerControl(self, position, rotation):
        try:

            if(self.type == "Servo"):

                topic_names = ["goal_axis1", "goal_axis2"]

                publishers = []

                for topic in IMPORT_TYPES[self.type]["control"]["topic"]:
                    publishers.append(self.node.create_publisher(self.classControl,
                                          self.node_name+"/"+topic)
                                     )

                msgs = [self.classControl(), self.classControl()]
                msgs[0].position = position
                msgs[0].velocity = self.rated_speed
                msgs[1].position = rotation
                msgs[1].velocity = self.rated_speed

                publishers[0].publish(msgs[0])
                publishers[1].publish(msgs[1])

                print("joint Axis1 goal:", "%.3f" % position, "Axis2 goal:", "%.3f" % rotation)

                for pub in publishers:
                    self.node.destroy_publisher(pub)

                ## With Actions
                # topic_names = ["trajectory_axis1","trajectory_axis2"]
                # clients = [
                #         ActionClient(self.node, self.classControl, topic_names[0]),
                #         ActionClient(self.node, self.classControl, topic_names[1])
                # ]
                #
                # responded = [False, False]
                #
                # counter = 10
                # while self.keepThreads and all(responded) and counter > 0:
                #     responded[0] = responded[0] or clients[0].wait_for_service(timeout_sec=0.5)
                #     responded[1] = responded[1] or clients[1].wait_for_service(timeout_sec=0.5)
                #     if(counter%2 == 0):
                #         self.node.get_logger().info('Trajectory action not available, waiting again...')
                #     counter -= 1
                #
                # if counter <=0 or not self.keepThreads:
                #     self.node.get_logger().info('Trajectory action not available, please try again...')
                #     sys.exit()
                #
                # goals = [self.classControl.Goal(), self.classControl.Goal()]
                # goals[0].position = position
                # goals[0].velocity = 1
                # goals[1].position = rotation
                # goals[1].velocity = 1
                #
                # futures = [clients[0].send_goal_async(goal[0]), clients[1].send_goal_async(goal[1])]
                #
                # while(futures[0].result() is None or futures[1].result() is None):
                #     time.sleep(0.1)
                #

            else:
                topic_name = "goal"
                client = self.node.create_client(self.classControl, self.node_name+"/"+topic_name)

                counter = 10
                while self.keepThreads and not client.wait_for_service(timeout_sec=0.5) and counter > 0:
                    if(counter%2 == 0):
                        self.node.get_logger().info(topic_name+' service not available, waiting again...')
                    counter -= 1

                if counter <=0 or not self.keepThreads:
                    self.node.get_logger().info(topic_name+' service not available, please try again...')
                    sys.exit()

                req = self.classControl.Request()
                req.goal_linearposition = position
                req.goal_angularposition = rotation

                print('moveGripper', "%.3f" % req.goal_linearposition, "%.3f" % req.goal_angularposition)

                future = client.call_async(req)

                while(future.result() is None):
                    time.sleep(0.1)

                print(future.result())

                self.node.destroy_client(client)

            sys.exit()
        except:
            raise

    #########################################################
    # Logic                                                 #
    #########################################################

    # Start threads if selected node is valid
    def connectToNode(self):
        threadID = threading.Thread(target=self.workerID)
        threadID.start()

        # Spinning thread
        spinner = threading.Thread(target=self.workerSpinner)
        spinner.start()

    # Latch onto component specific topics and services
    def latchToNode(self):

        threadSpecs = threading.Thread(target=self.workerSpecs)
        threadSpecs.start()

        threadSimulation = threading.Thread(target=self.workerSimulation)
        threadSimulation.start()

        threadState = threading.Thread(target=self.workerState)
        threadState.start()

        ## TODO: finalize thread migration
        # threadSimulation = threading.Thread(target=self.workerSimulation)
        # threadSimulation.start()

    # Refreshes list of nodes on node selection combobox
    def listNodes(self):
        try:

            # self.cleanup()
            nodes = self.node.get_node_names_and_namespaces()
            self.node_list = []

            self.wSelNode.blockSignals(True)
            self.wSelNode.clear()
            self.wSelNode.addItem("")
            for node in nodes:
                if(not node[0]==self.node.get_name() and
                    node[0] not in self.node_list and
                    "hrim_" in node[0]):
                    self.node_list.append(node[0])
            self.wSelNode.addItems(self.node_list)
            self.wSelNode.update()
            self.wSelNode.blockSignals(False)
        except:
            raise

    # ROS2 related cleanup when losing connection to a node
    def cleanup(self):

        self.executor.shutdown()
        time.sleep(0.05)

        self.node.destroy_node()
        time.sleep(0.05)

        self.node = rclpy.create_node('hrim_control_'+str(threading.get_ident()), context=self.context)

        self.executor = localExecutor(context=self.context)
        self.executor.add_node(self.node)

    def setIdInfo(self, result):

        global IMPORT_TYPES

        if result.device_name == "":
            self.type =  "Servo"
        else:
            self.type = result.device_name

        if(self.type == "Gripper"):
            self.withAction = False
        if(self.type == "Servo"):
            self.withAction = True

        self.classSpecs = getattr(importlib.import_module(
            IMPORT_TYPES[self.type]["specs"]["package"]),
            IMPORT_TYPES[self.type]["specs"]["class"]
        )
        self.classState = getattr(importlib.import_module(
            IMPORT_TYPES[self.type]["state"]["package"]),
            IMPORT_TYPES[self.type]["state"]["class"]
        )
        self.classControl = getattr(importlib.import_module(
            IMPORT_TYPES[self.type]["control"]["package"]),
            IMPORT_TYPES[self.type]["control"]["class"]
        )

        self.latchToNode()

        self.handleId(result)

    # 3D model zip decompression into temporary files
    def decompress(self):
        # Get bytestring from message
        data = b''.join(self.model)

        # Check if tmp path exists, delete it if it does
        path = os.path.join(os.getcwd(), "tmp")
        if os.path.exists(path):
            shutil.rmtree(os.path.join(os.getcwd(),"tmp"))
        os.makedirs(path)

        # Write compressed file
        filename = "description.zip"
        out_file = open(os.path.join(path, filename), "wb") # open for [w]riting as [b]inary
        out_file.write(data)
        out_file.close()

        # Unpack compressed file's contents without a file structure
        with zipfile.ZipFile(os.path.join(path, filename), "r") as zip_file:
            for member in zip_file.namelist():
                name = os.path.basename(member)
                # skip directories
                if not name:
                    continue

                # copy file (taken from zipfile's extract)
                source = zip_file.open(member)
                target = open(os.path.join(path, name), "wb")
                with source, target:
                    shutil.copyfileobj(source, target)

        # Delete compressed file
        os.remove(os.path.join(path, filename))

        urdfFile = None
        for file in os.listdir(path):
            if file.endswith(".urdf"):
                urdfFile = os.path.join(os.getcwd(), path, file)

        tmp_file = open(urdfFile, "r")
        tmp_contents = tmp_file.read()
        tmp_file.close()

        os.remove(urdfFile)

        processed_urdf = re.sub(r'( *<mesh filename=")package://.*(/[^/]*.dae)(".*)', r'\g<1>file://'+os.path.join(os.getcwd(), path)+'\g<2>\g<3>', tmp_contents)
        processed_urdf = re.sub(r'(<joint.*type=")(.*)(">)', r'\g<1>fixed\g<3>', processed_urdf)

        with open(os.path.join(path, "temporary.urdf"), "w") as myFile:
            myFile.write(processed_urdf)


        with open("config.rviz", "r") as myFile:
            base_rviz = myFile.read()

        processed_rviz = re.sub(r'%FIXED_FRAME%', self.model_frame, base_rviz)
        processed_rviz = re.sub(r'%URDF_FILE%', os.path.join(path, "temporary.urdf"), processed_rviz)

        with open(os.path.join("tmp", "temporary.rviz"), "w") as myFile:
            myFile.write(processed_rviz)

        os.environ["HRIM_TMP_URDF"] = os.path.join(path, "temporary.urdf")
        os.environ["HRIM_TMP_RVIZ"] = os.path.join(path, "temporary.rviz")

        # self.stateGroup.setTabEnabled(2, True)

        # self.rviz_proc = subprocess.Popen("rviz2 -d "+os.path.join("tmp", "temporary.rviz"), shell = True)
        # self.rviz_proc = subprocess.Popen("ros2 launch my_dummy_robot_bringup trying.launch.py", shell = True)

    def launchRviz(self):

        process = Process(target=fastLaunch.launchMe)
        process.daemon = True
        process.start()
        # # save current position
        # tmpPosition = self.pos()
        #
        # # hide main window as it'll freeze
        # self.hide()
        #
        # # launch rviz2 + robot state publisher
        # fastLaunch.launchMe()
        #
        # # TODO: doesn't seem to work if the previous values aren't modified
        # # when rviz closes, reposition ourselves on previous location
        # self.move(tmpPosition.x()+1, tmpPosition.y())
        #
        # self.show()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = HrimGripperControl(app)
    sys.exit(app.exec_())
