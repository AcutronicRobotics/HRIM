# Qt5
import importlib  # dynamic imports
import math  # for math.nan
import os
import re  # urdf editing through regex
import shutil
# system
import sys
import threading
import time
import zipfile  # zip (de)compression for 3D models

# component rviz visualization
import fastLaunch
# ROS2
import rclpy
from PyQt5 import QtCore, QtGui
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import (QWidget, QSlider, QGridLayout, QLayout,
                             QPushButton, QStackedWidget,
                             QLabel, QComboBox, QApplication, QTabWidget,
                             QVBoxLayout, QLineEdit)
from billiard import Process
from hrim_generic_srvs.srv import ID, Simulation3D
from rclpy.executors import SingleThreadedExecutor as localExecutor
from rclpy.qos import qos_profile_sensor_data as rclpy_qos

# noinspection PyPep8
IMPORT_TYPES = {
    "Gripper": {
        "specs": {"package": "hrim_actuator_gripper_srvs.srv",
                  "class": "SpecsFingerGripper",
                  "topic": "specs"
                  },
        "state": {"package": "hrim_actuator_gripper_msgs.msg",
                  "class": "StateFingerGripper",
                  "topic": ["fingerstate"],
                  "property": {
                      "Position": "linear_position",
                      "Rotation": "angular_position"
                  }
                  },
        "control": {"package": "hrim_actuator_gripper_srvs.srv",
                    "class": "ControlFinger",
                    "type": "service",
                    "topic": ["fingercontrol"]
                    }
    },
    "Servo": {
        "specs": {"package": "hrim_actuator_rotaryservo_srvs.srv",
                  "class": "SpecsRotaryServo",
                  "topic": "specs"
                  },
        "state": {"package": "hrim_actuator_rotaryservo_msgs.msg",
                  "class": "StateRotaryServo",
                  "topic": ["state_axis1", "state_axis2"],
                  "property": {
                      "Position": "position"
                  }
                  },
        "control": {"package": "hrim_actuator_rotaryservo_msgs.msg",
                    "class": "GoalRotaryServo",
                    "type": "topic",
                    "topic": ["goal_axis1", "goal_axis2"]
                    }
    }
}


class ComboBox(QComboBox):
    popupAboutToBeShown = QtCore.pyqtSignal()

    def show_popup(self):
        self.popupAboutToBeShown.emit()
        super(ComboBox, self).show_popup()


def launch_rviz():
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


class HrimGripperControl(QWidget):

    # Class initializer
    def __init__(self, main_app):
        self.wLblNodeCount = QLabel("Node")
        self.wSelNode = ComboBox()
        self.wBtnGripperPos = QPushButton("Position")
        self.wInGripperPos = QLineEdit()
        self.wSldGripperPos = QSlider(QtCore.Qt.Horizontal, self)
        self.wLblGripperPosMax = QLabel("0")
        self.wBtnGripperRot = QPushButton("Rotation")
        self.wInGripperRot = QLineEdit()
        self.wSldGripperRot = QSlider(QtCore.Qt.Horizontal, self)
        self.wLblGripperRotMax = QLabel("0")
        self.wStackGripper = QWidget()
        self.wBtnJointAxis1 = QPushButton("Axis 1")
        self.wInJointAxis1 = QLineEdit()
        self.wSldJointAxis1 = QSlider(QtCore.Qt.Horizontal, self)
        self.wLblJointAxis1Max = QLabel("0")
        self.wBtnJointAxis2 = QPushButton("Axis 2")
        self.wInJointAxis2 = QLineEdit()
        self.wSldJointAxis2 = QSlider(QtCore.Qt.Horizontal, self)
        self.wLblJointAxis2Max = QLabel("0")
        self.wBtnJointStreaming = QPushButton("Stream goal")
        self.wStackJoint = QWidget()
        self.layControlInput = QStackedWidget(self)
        self.wBtnGo = QPushButton("Send order")
        self.layControl = QWidget()
        self.layTabs = QTabWidget()
        self.w_lblId_category_value = QLabel("")
        self.wLblIdTypeValue = QLabel("")
        self.wLblIdHrimValue = QLabel("")
        self.wLblIdHrosValue = QLabel("")
        self.wTabState = QWidget()
        self.wLblStateFirst = QLabel("")
        self.wLblStateFirstValue = QLabel("0.000")
        self.wLblStateSecond = QLabel("")
        self.wLblStateSecondValue = QLabel("0.000")
        self.wLblPowerPowerConsumptionValue = QLabel("0.000")
        self.wLblPowerCurrentConsumptionValue = QLabel("0.00000")
        self.wLblPowerVoltageValue = QLabel("0.000")
        self.maxRotation = float("%.3f" % result.max_angle)
        self.minJoint = float("%.3f" % result.range_min)
        self.maxJoint = float("%.3f" % result.range_max)
        self.node_list = []
        self.type = result.device_name
        try:
            super().__init__()

            self.app = main_app

            self.context = rclpy.context.Context()
            rclpy.init(context=self.context)

            self.node = rclpy.create_node('hrim_control_' +
                                          str(threading.get_ident()),
                                          context=self.context)

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

            self.init_ui()

            watcher = threading.Thread(target=self.worker_watcher)
            watcher.start()

            self.show()

        except KeyboardInterrupt:
            self.close_event()
            raise
        except Exception as e:
            raise e

    # UI initialization
    def init_ui(self):

        # self.setFixedSize(430,300)
        self.setFixedWidth(460)

        v_box = QVBoxLayout()
        v_box.setAlignment(QtCore.Qt.AlignTop)
        # resize to minimal space on content changes
        v_box.setSizeConstraint(QLayout.SetFixedSize)

        self.setLayout(v_box)

        # ## Node selection

        # Thread count label

        # Node select combobox
        self.wSelNode.addItem("")
        self.wSelNode.activated[int].connect(self.node_selected)
        self.wSelNode.popupAboutToBeShown.connect(self.list_nodes)

        # Disconnect button
        w_btn_refresh = QPushButton("")
        w_btn_refresh.setIcon(QIcon('img/refresh.png'))
        w_btn_refresh.clicked.connect(self.clicked_on_refresh)

        # Node selection widget
        lay_select = QWidget()
        lay_select.layout = QGridLayout()
        lay_select.layout.addWidget(self.wLblNodeCount, 0, 0)
        lay_select.layout.addWidget(self.wSelNode, 0, 1, 1, 10)
        lay_select.layout.addWidget(w_btn_refresh, 0, 11)
        lay_select.setLayout(lay_select.layout)
        lay_select.setFixedSize(400, 50)

        v_box.addWidget(lay_select)

        # ## Control

        regex = QtCore.QRegExp("^-?[0-9]*[.]?[0-9]*$")
        validator = QtGui.QRegExpValidator(regex)

        # # Gripper

        # Gripper position
        self.wBtnGripperPos.setCheckable(True)
        self.wBtnGripperPos.setEnabled(False)
        self.wBtnGripperPos.clicked[bool]. \
            connect(self.w_btn_gripper_pos_toggle)

        self.wInGripperPos.setFixedWidth(60)

        self.wInGripperPos.setValidator(validator)
        self.wInGripperPos.setText("0.000")
        self.wInGripperPos.returnPressed.connect(self.w_in_gripper_pos_change)
        self.wInGripperPos.setEnabled(False)

        self.wSldGripperPos.setMinimum(0)
        self.wSldGripperPos.setMaximum(1000)
        self.wSldGripperPos.setFocusPolicy(QtCore.Qt.NoFocus)
        self.wSldGripperPos.setGeometry(30, 40, 100, 30)
        self.wSldGripperPos.valueChanged[int].connect(
            self.w_s_id_gripper_pos_change)
        self.wSldGripperPos.setEnabled(False)

        self.wLblGripperPosMax.setEnabled(False)

        # Gripper rotation
        self.wBtnGripperRot.setCheckable(True)
        self.wBtnGripperRot.setEnabled(False)
        self.wBtnGripperRot.clicked[bool]. \
            connect(self.w_btn_gripper_rot_toggle)

        self.wInGripperRot.setFixedWidth(60)

        self.wInGripperRot.setValidator(validator)
        self.wInGripperRot.setText("0.000")
        self.wInGripperRot.returnPressed.connect(self.w_in_gripper_rot_change)
        self.wInGripperRot.setEnabled(False)

        self.wSldGripperRot.setMinimum(0)
        self.wSldGripperRot.setMaximum(1000)
        self.wSldGripperRot.setFocusPolicy(QtCore.Qt.NoFocus)
        self.wSldGripperRot.setGeometry(30, 40, 100, 30)
        self.wSldGripperRot.valueChanged[int].connect(
            self.w_sld_gripper_rot_change)
        self.wSldGripperRot.setEnabled(False)

        self.wLblGripperRotMax.setEnabled(False)

        # Gripper control stack
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
        self.wStackGripper.setFixedSize(400, 100)

        # # Joint

        # Joint axis 1
        self.wBtnJointAxis1.setCheckable(True)
        self.wBtnJointAxis1.setEnabled(False)
        self.wBtnJointAxis1.clicked[bool]. \
            connect(self.w_btn_joint_axis1_toggle)

        self.wInJointAxis1.setFixedWidth(60)

        self.wInJointAxis1.setValidator(validator)
        self.wInJointAxis1.setText("0.000")
        self.wInJointAxis1.returnPressed.connect(self.w_in_joint_axis1_change)
        self.wInJointAxis1.setEnabled(False)

        self.wSldJointAxis1.setMinimum(0)
        self.wSldJointAxis1.setMaximum(1000)
        self.wSldJointAxis1.setValue(500)
        self.wSldJointAxis1.setFocusPolicy(QtCore.Qt.NoFocus)
        self.wSldJointAxis1.setGeometry(30, 40, 100, 30)
        self.wSldJointAxis1.valueChanged[int].connect(
            self.w_sld_joint_axis1_change)
        self.wSldJointAxis1.setEnabled(False)

        self.wLblJointAxis1Max.setEnabled(False)

        # Joint rotation
        self.wBtnJointAxis2.setCheckable(True)
        self.wBtnJointAxis2.setEnabled(False)
        self.wBtnJointAxis2.clicked[bool]. \
            connect(self.w_btn_joint_axis2_toggle)

        self.wInJointAxis2.setFixedWidth(60)

        self.wInJointAxis2.setValidator(validator)
        self.wInJointAxis2.setText("0.000")
        self.wInJointAxis2.returnPressed.connect(self.w_in_joint_axis2_change)
        self.wInJointAxis2.setEnabled(False)

        self.wSldJointAxis2.setMinimum(0)
        self.wSldJointAxis2.setMaximum(1000)
        self.wSldJointAxis2.setValue(500)
        self.wSldJointAxis2.setFocusPolicy(QtCore.Qt.NoFocus)
        self.wSldJointAxis2.setGeometry(30, 40, 100, 30)
        self.wSldJointAxis2.valueChanged[int].connect(
            self.w_sld_joint_axis2_change)
        self.wSldJointAxis2.setEnabled(False)

        self.wLblJointAxis2Max.setEnabled(False)

        # Joint goal streaming button
        self.wBtnJointStreaming.setCheckable(True)
        self.wBtnJointStreaming.setEnabled(False)
        self.wBtnJointStreaming.clicked[bool].connect(
            self.w_btn_joint_streaming_toggle)

        # Joint control stack
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
        self.wStackJoint.setFixedSize(400, 100)

        # # Control input section
        self.layControlInput.addWidget(self.wStackGripper)
        self.layControlInput.addWidget(self.wStackJoint)
        # self.layControlInput.setFixedSize(400,300)

        # # Send order button
        self.wBtnGo.clicked.connect(self.w_btn_go_clicked)
        self.wBtnGo.setEnabled(False)

        # # Overall control section
        self.layControl.layout = QGridLayout()
        self.layControl.layout.addWidget(self.layControlInput)
        self.layControl.layout.addWidget(self.wBtnGo)
        self.layControl.setLayout(self.layControl.layout)
        self.layControl.hide()

        v_box.addWidget(self.layControl)

        # ## Component information tabs

        # ID tab
        w_tab_id = QWidget()
        w_tab_id.layout = QGridLayout()

        w_lbl_id_category = QLabel("Device kind:")
        w_lbl_id_type = QLabel("Device name:")
        w_lbl_id_hrim = QLabel("HRIM version:")
        w_lbl_id_hros = QLabel("HROS version:")

        w_tab_id.layout.addWidget(w_lbl_id_category, 0, 0)
        w_tab_id.layout.addWidget(self.w_lblId_category_value, 0, 1)
        w_tab_id.layout.addWidget(w_lbl_id_type, 1, 0)
        w_tab_id.layout.addWidget(self.wLblIdTypeValue, 1, 1)
        w_tab_id.layout.addWidget(w_lbl_id_hrim, 2, 0)
        w_tab_id.layout.addWidget(self.wLblIdHrimValue, 2, 1)
        w_tab_id.layout.addWidget(w_lbl_id_hros, 3, 0)
        w_tab_id.layout.addWidget(self.wLblIdHrosValue, 3, 1)

        w_tab_id.setLayout(w_tab_id.layout)

        # # State tab
        self.wTabState.layout = QGridLayout()

        self.wTabState.layout.addWidget(self.wLblStateFirst, 0, 0)
        self.wTabState.layout.addWidget(self.wLblStateFirstValue, 0, 1)
        self.wTabState.layout.addWidget(self.wLblStateSecond, 1, 0)
        self.wTabState.layout.addWidget(self.wLblStateSecondValue, 1, 1)

        self.wTabState.setLayout(self.wTabState.layout)

        # # Power tab
        w_tab_power = QWidget()
        w_tab_power.layout = QGridLayout()

        w_lbl_power_power_consumption = QLabel("Power consumption:")
        w_lbl_power_current_consumption = QLabel("Current consumption:")
        w_lbl_power_voltage = QLabel("Voltage:")

        w_tab_power.layout.addWidget(w_lbl_power_voltage, 0, 0)
        w_tab_power.layout.addWidget(self.wLblPowerVoltageValue, 0, 1)
        w_tab_power.layout.addWidget(w_lbl_power_power_consumption, 1, 0)
        w_tab_power.layout.addWidget(self.wLblPowerPowerConsumptionValue, 1, 1)
        w_tab_power.layout.addWidget(w_lbl_power_current_consumption, 2, 0)
        w_tab_power.layout.addWidget(self.wLblPowerCurrentConsumptionValue, 2,
                                     1)

        w_tab_power.setLayout(w_tab_power.layout)

        w_tab3_d = QWidget()
        w_tab3_d.layout = QGridLayout()

        w_btn3_d_rviz = QPushButton("rviz2")
        w_btn3_d_rviz.clicked.connect(launch_rviz)

        w_tab3_d.layout.addWidget(w_btn3_d_rviz, 0, 0)

        w_tab3_d.setLayout(w_tab3_d.layout)

        # # Add tabs
        self.layTabs.addTab(w_tab_id, "ID")
        self.layTabs.addTab(self.wTabState, "State")
        self.layTabs.addTab(w_tab_power, "Power")
        self.layTabs.addTab(w_tab3_d, "3D")
        self.layTabs.hide()

        v_box.addWidget(self.layTabs)

    # Closing event override
    @staticmethod
    def close_event():
        try:

            # Signal worker threads should stop
            self.keepThreads = False

            # Give time to threads to stop
            time.sleep(0.1)

            if os.path.exists(os.path.join(os.getcwd(), "tmp")):
                shutil.rmtree(os.path.join(os.getcwd(), "tmp"))

            # ROS2 cleanup
            self.executor.shutdown()
            self.node.destroy_node()
            rclpy.shutdown(context=self.context)

            sys.exit(0)
        except Exception as e:
            raise e

    #########################################################
    # UI interaction handlers                               #
    #########################################################

    def node_selected(self, value):
        try:

            self.validNode = False
            counter = 0
            while threading.active_count() > 2 and counter < 100:
                time.sleep(0.01)

            self.reset_all()

            if value > 0 and counter < 100:
                self.node_name = self.node_list[value - 1]
                if "/" + self.node_name + "/specs" in \
                        dict(self.node.get_service_names_and_types()):
                    self.validNode = True
                    self.connect_to_node()
                else:
                    self.validNode = False
                    print("Not a valid HRIM gripper node")
        except Exception as e:
            raise e

    def clicked_on_refresh(self):
        try:
            self.validNode = False

            self.wSelNode.blockSignals(True)
            self.wSelNode.setCurrentIndex(0)
            self.wSelNode.blockSignals(False)

            self.cleanup()

            # self.resetUI()
            self.reset_all()

        except Exception as e:
            raise e

    def w_btn_go_clicked(self):
        try:
            arg1 = None
            arg2 = None
            if self.type == "Gripper":
                arg1 = self.firstValue if self.wSldGripperPos.isEnabled() \
                    else math.nan
                arg2 = self.secondValue if self.wSldGripperRot.isEnabled() \
                    else math.nan
            if self.type == "Servo":
                arg1 = self.firstValue if self.wSldJointAxis1.isEnabled() \
                    else math.nan
                arg2 = self.secondValue if self.wSldJointAxis2.isEnabled() \
                    else math.nan

            t = threading.Thread(target=self.worker_control, args=[arg1, arg2])
            t.start()
        except Exception as e:
            raise e

    # # Gripper
    def w_in_gripper_pos_change(self):
        try:
            value = self.wInGripperPos.text()
            if len(value) < 1 or value in [".", "-"] or float(value) == 0.0:
                val = 0.0
                self.wSldGripperPos.setValue(0)
            else:
                val = float(value)
                self.wSldGripperPos.setValue((val * 1000) / self.maxPosition)
            self.firstValue = val
        except Exception as e:
            raise e

    def w_s_id_gripper_pos_change(self, value):
        try:
            self.firstValue = float((self.maxPosition / 1000) * value)
            self.wInGripperPos.setText("%.3f" % self.firstValue)
            self.wInGripperPos.update()
        except Exception as e:
            raise e

    def w_in_gripper_rot_change(self):
        try:
            value = self.wInGripperRot.text()
            if len(value) < 1 or value in [".", "-"] or float(value) == 0.0:
                val = 0.0
                self.wSldGripperRot.setValue(0)
            else:
                val = float(value)
                self.wSldGripperRot.setValue((val * 1000) / self.maxRotation)
            self.secondValue = val
        except Exception as e:
            raise e

    def w_sld_gripper_rot_change(self, value):
        try:
            self.secondValue = float((self.maxRotation / 1000) * value)
            self.wInGripperRot.setText("%.3f" % self.secondValue)
            self.wInGripperRot.update()
        except Exception as e:
            raise e

    def w_btn_gripper_pos_toggle(self, is_checked):
        try:
            self.wInGripperPos.setEnabled(is_checked)
            self.wLblGripperPosMax.setEnabled(is_checked)
            self.wSldGripperPos.setEnabled(is_checked)

            self.wBtnGo.setEnabled(any([is_checked,
                                        self.wBtnGripperRot.isChecked()]))
        except Exception as e:
            raise e

    def w_btn_gripper_rot_toggle(self, is_checked):
        try:
            self.wInGripperRot.setEnabled(is_checked)
            self.wLblGripperRotMax.setEnabled(is_checked)
            self.wSldGripperRot.setEnabled(is_checked)

            self.wBtnGo.setEnabled(any([is_checked,
                                        self.wBtnGripperPos.isChecked()]))
        except Exception as e:
            raise e

    # # Joints
    def w_in_joint_axis1_change(self):
        try:
            value = self.wInJointAxis1.text()
            if len(value) < 1 or value in [".", "-"] or float(value) == 0.0:
                val = 0.0
                self.wSldJointAxis1.blockSignals(True)
                self.wSldJointAxis1.setValue(500)
                self.wSldJointAxis1.blockSignals(False)
            else:
                val = float(value)
                self.wSldJointAxis1.blockSignals(True)
                self.wSldJointAxis1.setValue(1000 / (
                        (self.maxJoint - self.minJoint) /
                        (val - self.minJoint)))
                self.wSldJointAxis1.blockSignals(False)
            self.firstValue = val

        except Exception as e:
            raise e

    def w_sld_joint_axis1_change(self, value):
        try:
            joint_range = self.maxJoint - self.minJoint
            self.firstValue = float(
                (joint_range / 1000) * value) + self.minJoint

            self.wInJointAxis1.blockSignals(True)
            self.wInJointAxis1.setText("%.3f" % self.firstValue)
            self.wInJointAxis1.update()
            self.wInJointAxis1.blockSignals(False)

        except Exception as e:
            raise e

    def w_in_joint_axis2_change(self):
        try:
            value = self.wInJointAxis2.text()
            if len(value) < 1 or value in [".", "-"] or float(value) == 0.0:
                val = 0.0

                self.wSldJointAxis2.blockSignals(True)
                self.wSldJointAxis2.setValue(500)
                self.wSldJointAxis2.blockSignals(False)
            else:
                val = float(value)
                self.wSldJointAxis2.blockSignals(True)
                self.wSldJointAxis2.setValue(1000 / (
                        (self.maxJoint - self.minJoint) /
                        (val - self.minJoint)))
                self.wSldJointAxis2.blockSignals(False)
            self.secondValue = val

        except Exception as e:
            raise e

    def w_sld_joint_axis2_change(self, value):
        try:
            range_values = self.maxJoint - self.minJoint
            self.secondValue = \
                float((range_values / 1000) * value) + self.minJoint

            self.wInJointAxis2.blockSignals(True)
            self.wInJointAxis2.setText("%.3f" % self.secondValue)
            self.wInJointAxis2.update()
            self.wInJointAxis2.blockSignals(False)

        except Exception as e:
            raise e

    def w_btn_joint_axis1_toggle(self, is_checked):
        try:
            self.wInJointAxis1.setEnabled(is_checked)
            self.wLblJointAxis1Max.setEnabled(is_checked)
            self.wSldJointAxis1.setEnabled(is_checked)

            self.wBtnGo.setEnabled(any([is_checked,
                                        self.wBtnJointAxis2.isChecked()]))
            self.wBtnJointStreaming.setEnabled(any([is_checked,
                                                    self.wBtnJointAxis2.
                                                   isChecked()]))
            if not any([is_checked, self.wBtnJointAxis2.isChecked()]):
                self.wBtnJointStreaming.setChecked(False)
        except Exception as e:
            raise e

    def w_btn_joint_axis2_toggle(self, is_checked):
        try:
            self.wInJointAxis2.setEnabled(is_checked)
            self.wLblJointAxis2Max.setEnabled(is_checked)
            self.wSldJointAxis2.setEnabled(is_checked)

            self.wBtnGo.setEnabled(any([is_checked, self.wBtnJointAxis1.
                                       isChecked()]))
            self.wBtnJointStreaming.setEnabled(any([is_checked,
                                                    self.wBtnJointAxis1.
                                                   isChecked()]))
            if not any([is_checked, self.wBtnJointAxis1.isChecked()]):
                self.wBtnJointStreaming.setChecked(False)
        except Exception as e:
            raise e

    def w_btn_joint_streaming_toggle(self, is_checked):
        try:
            self.streaming = is_checked
            if is_checked:
                thread_streaming = threading.Thread(
                    target=self.worker_streaming)
                thread_streaming.start()
        except Exception as e:
            raise e

    #########################################################
    # UI manipulation                                       #
    #########################################################

    def reset_all(self):

        # for x in range(0, len(self.state_subs)):
        #     self.node.destroy_subscription(self.state_subs[x])

        self.state_subs = []
        self.streaming_pubs = []
        self.streaming_msgs = []

        try:
            self.reset_ui()
            self.reset_counters()
            # self.resetState()
            # self.resetID()
        except Exception as e:
            raise e

    def reset_ui(self):
        try:
            self.reset_control()
            self.layTabs.setCurrentIndex(0)
            self.layTabs.setTabEnabled(2, False)
            self.layTabs.setTabEnabled(3, False)
        except Exception as e:
            raise e

    def reset_control(self):
        try:

            self.wBtnGripperPos.blockSignals(True)
            self.wInGripperPos.blockSignals(True)
            self.wSldGripperPos.blockSignals(True)

            self.wBtnGripperPos.setChecked(False)
            self.w_btn_gripper_pos_toggle(False)
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
            self.w_btn_gripper_rot_toggle(False)
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
            self.w_btn_joint_axis1_toggle(False)
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
            self.w_btn_joint_axis2_toggle(False)
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

        except Exception as e:
            raise e

    def reset_counters(self):
        try:
            self.stateCounter = 0
            self.firstValue = 0.0
            self.secondValue = 0.0
        except Exception as e:
            raise e

    def handle_id(self, result):
        hrim_types = ["Communication", "Sensor", "Actuator",
                      "Cognition", "UI", "Power", "Composite"]
        self.w_lblId_category_value.setText(hrim_types[result.device_kind_id])
        self.wLblIdTypeValue.setText(result.device_name)
        self.wLblIdHrimValue.setText(result.hrim_version)
        self.wLblIdHrosValue.setText(result.hros_version)

    def handle_specs(self, result):

        if self.type == "Gripper":
            self.maxPosition = float("%.3f" % result.max_length)
            self.wLblGripperPosMax.setText(str(self.maxPosition))
            self.wBtnGripperPos.setEnabled(True)

            self.wLblStateFirst.setText("Position:")
            self.wLblStateSecond.setText("Rotation:")

            if result.max_angle > 0:
                print("got angle")
                self.wLblGripperRotMax.setText(str(self.maxRotation))
                self.wBtnGripperRot.setEnabled(True)

            self.layControlInput.setCurrentIndex(0)

        elif self.type == "Servo":
            self.rated_speed = result.rated_speed

            self.wLblJointAxis1Max.setText(str(self.maxJoint))
            self.wLblJointAxis2Max.setText(str(self.maxJoint))

            self.wBtnJointAxis1.setEnabled(True)
            self.wBtnJointAxis2.setEnabled(True)

            self.wLblStateFirst.setText("Axis 1:")
            self.wLblStateSecond.setText("Axis 2:")

            self.layControlInput.setCurrentIndex(1)

        self.layControl.show()
        self.layTabs.show()

    def callback_finger_state(self, data):
        try:
            self.stateCounter = 0
            self.wLblStateFirstValue.setText("%.3f" % data.linear_position)
            self.wLblStateSecondValue.setText("%.3f" % data.angular_position)
        except Exception as e:
            raise e

    def callback_servo_first_state(self, data):
        try:
            self.stateCounter = 0
            self.wLblStateFirstValue.setText("%.3f" % data.position)
        except Exception as e:
            raise e

    def callback_servo_second_state(self, data):
        try:
            self.stateCounter = 0
            self.wLblStateSecondValue.setText("%.3f" % data.position)
        except Exception as e:
            raise e

    def callback_power(self, data):
        try:
            if not self.layTabs.isTabEnabled(2):
                self.layTabs.setTabEnabled(2, True)
            self.wLblPowerVoltageValue.setText("%.3f" % data.voltage)
            self.wLblPowerPowerConsumptionValue.setText("%.3f" %
                                                        data.power_consumption)
            self.wLblPowerCurrentConsumptionValue.setText("%.5f" %
                                                          data.
                                                          current_consumption)
        except Exception as e:
            raise e

    #########################################################
    # Thread workers                                        #
    #########################################################

    def worker_watcher(self):
        try:
            while self.keepThreads:
                self.wLblNodeCount.setText(str(threading.active_count() - 1))
                time.sleep(0.05)
            sys.exit()
        except Exception as e:
            raise e

    def worker_spinner(self):
        try:
            while self.keepThreads and self.validNode:
                # Any value for the spin_once timeout
                # other than 0 ends un crashing the interface
                self.executor.spin_once(0)
                time.sleep(0.01)
            sys.exit()
        except Exception as e:
            raise e

    # Identification service worker
    def worker_id(self):
        try:
            topic_name = "id"
            client = self.node.create_client(
                ID, self.node_name + "/" + topic_name)

            counter = 10
            while self.keepThreads and not \
                    client.wait_for_service(timeout_sec=0.5) and counter > 0:
                if counter % 2 == 0:
                    self.node.get_logger().\
                        info(topic_name +
                             ' service not available, waiting again...')
                counter -= 1

            if counter <= 0 or not self.keepThreads:
                self.node.get_logger().\
                    info(topic_name +
                         ' service not available, please try again...')
                sys.exit()

            future = client.call_async(ID.Request())

            counter = 10
            while future.result() is None and counter > 0:
                counter -= 1
                time.sleep(0.1)

            if counter <= 0:
                self.node.get_logger().info('Call to ' + topic_name +
                                            ' service timed out.')

            if future.result() is not None:
                self.set_id_info(future.result())
            else:
                self.node.get_logger().info('Service call failed %r' %
                                            (future.exception()))

            sys.exit()
        except Exception as e:
            raise e

    # Specs service worker
    def worker_specs(self):
        try:
            topic_name = "specs"

            client = self.node.create_client(
                self.classSpecs, "/" + self.node_name + "/" + topic_name)

            print("Creating service: ", self.node_name + "/" + topic_name,
                  self.classSpecs, )

            counter = 10

            while self.keepThreads and self.validNode and not \
                    client.wait_for_service(timeout_sec=1.0) and counter > 0:
                self.node.get_logger().info(
                    'service not available, waiting again...')
                counter -= 1

            if counter <= 0 or not self.keepThreads:
                self.node.get_logger().\
                    info(topic_name +
                         ' service not available, please try again...')
                sys.exit()

            future = client.call_async(self.classSpecs.Request())

            counter = 10
            while future.result() is None and counter > 0:
                counter -= 1
                time.sleep(0.1)

            if counter <= 0:
                self.node.get_logger().info('Call to ' + topic_name +
                                            ' service timed out.')

            if future.result() is not None:
                self.handle_specs(future.result())

            else:
                self.node.get_logger().info('Service call failed %r' %
                                            (future.exception()))

            sys.exit()
        except Exception as e:
            raise e

    def worker_simulation(self):
        try:
            topic_name = "module_3d"

            client = self.node.create_client(
                Simulation3D, self.node_name + "/" + topic_name)

            print("Simulation ", "/" + self.node_name + "/" + topic_name,
                  Simulation3D)

            counter = 10
            while self.keepThreads and self.validNode and not \
                    client.wait_for_service(timeout_sec=1.0) and counter > 0:
                self.node.get_logger().info(
                    'service not available, waiting again...')
                counter -= 1

            if counter <= 0 or not self.keepThreads:
                self.node.get_logger().\
                    info(topic_name +
                         ' service not available, please try again...')

            future = client.call_async(Simulation3D.Request())

            counter = 10
            while future.result() is None and counter > 0:
                counter -= 1
                time.sleep(0.1)

            if counter <= 0:
                self.node.get_logger().info('Call to ' + topic_name +
                                            ' service timed out.')

            if future.result() is not None:
                self.node.get_logger().info('Service call ok')

                self.model_frame = future.result().model_name
                self.model = future.result().model
                if len(self.model) > 0:
                    self.decompress()
                    self.layTabs.setTabEnabled(3, True)

            else:
                self.node.get_logger().info('Service call failed %r' %
                                            (future.exception()))

            sys.exit()
        except Exception as e:
            raise e

    def worker_state(self):
        try:
            for x in range(0, len(self.state_subs)):
                self.node.destroy_subscription(self.state_subs[x])

            self.state_subs = []

            if self.type == "Servo":
                print("Servo")

                self.state_subs.append(self.node.create_subscription(
                    self.classState, "/" + self.node_name +
                                     "/state_axis1",
                    self.callback_servo_first_state,
                    qos_profile=rclpy_qos
                ))
                print("/" + self.node_name + "/state_axis1", self.classState)

                self.state_subs.append(self.node.create_subscription(
                    self.classState, "/" + self.node_name +
                                     "/state_axis2",
                    self.callback_servo_second_state, qos_profile=rclpy_qos
                ))
                print("/" + self.node_name + "/state_axis2", self.classState)

            if self.type == "Gripper":
                print("Gripper")
                for topic in IMPORT_TYPES[self.type]["state"]["topic"]:
                    print("State: /" + self.node_name + "/" + topic,
                          self.classState)
                    self.state_subs.append(self.node.create_subscription(
                        self.classState, "/" + self.node_name + "/" + topic,
                        self.callback_finger_state)
                    )

            while (
                    self.keepThreads and self.validNode and
                    self.stateCounter < self.maxStateCounter):
                self.stateCounter += 1
                time.sleep(0.1)

            for x in range(0, len(self.state_subs)):
                self.node.destroy_subscription(self.state_subs[x])

            if self.keepThreads and self.validNode:
                print("State timeout")
                time.sleep(0.05)
                self.clicked_on_refresh()

            sys.exit()
        except Exception as e:
            raise e

    def worker_streaming(self):
        try:
            for x in range(0, len(self.streaming_pubs)):
                self.node.destroy_publisher(self.streaming_pubs[x])

            self.streaming_pubs = []
            self.streaming_msgs = []

            for topic in IMPORT_TYPES[self.type]["control"]["topic"]:
                self.streaming_pubs.append(
                    self.node.create_publisher(self.classControl,
                                               self.node_name + "/" + topic)
                )
                msg = self.classControl()
                msg.velocity = self.rated_speed
                self.streaming_msgs.append(msg)

            while self.streaming and self.keepThreads and self.validNode:
                self.streaming_msgs[
                    0].position = self.firstValue if self.wSldJointAxis1.\
                    isEnabled() else math.nan
                self.streaming_msgs[
                    1].position = self.secondValue if self.wSldJointAxis2.\
                    isEnabled() else math.nan
                for index in range(0, len(self.streaming_pubs)):
                    self.streaming_pubs[index].publish(
                        self.streaming_msgs[index])
                time.sleep(0.1)

            for x in range(0, len(self.streaming_pubs)):
                self.node.destroy_publisher(self.streaming_pubs[x])

            sys.exit()
        except Exception as e:
            raise e

    def worker_control(self, position, rotation):
        try:

            if self.type == "Servo":

                publishers = []

                for topic in IMPORT_TYPES[self.type]["control"]["topic"]:
                    publishers.append(
                        self.node.create_publisher(self.classControl,
                                                   self.node_name + "/" +
                                                   topic)
                    )

                msgs = [self.classControl(), self.classControl()]
                msgs[0].position = position
                msgs[0].velocity = self.rated_speed
                msgs[1].position = rotation
                msgs[1].velocity = self.rated_speed

                publishers[0].publish(msgs[0])
                publishers[1].publish(msgs[1])

                print("joint Axis1 goal:", "%.3f" % position, "Axis2 goal:",
                      "%.3f" % rotation)

                for pub in publishers:
                    self.node.destroy_publisher(pub)

                # # With Actions
                # topic_names = ["trajectory_axis1","trajectory_axis2"]
                # clients = [
                #         ActionClient(self.node,
                # self.classControl, topic_names[0]),
                #         ActionClient(self.node,
                # self.classControl, topic_names[1])
                # ]
                #
                # responded = [False, False]
                #
                # counter = 10
                # while self.keepThreads and all(responded) and counter > 0:
                #     responded[0] = responded[0] or clients[0].wait_for_
                # service(timeout_sec=0.5)
                #     responded[1] = responded[1] or clients[1].wait_for_
                # service(timeout_sec=0.5)
                #     if(counter%2 == 0):
                #         self.node.get_logger().info('Trajectory
                # action not available,
                # waiting again...')
                #     counter -= 1
                #
                # if counter <=0 or not self.keepThreads:
                #     self.node.get_logger().info('Trajectory action not
                # available, please try again...')
                #     sys.exit()
                #
                # goals = [self.classControl.Goal(), self.classControl.Goal()]
                # goals[0].position = position
                # goals[0].velocity = 1
                # goals[1].position = rotation
                # goals[1].velocity = 1
                #
                # futures = [clients[0].send_goal_async(goal[0]), clients[1].
                # send_goal_async(goal[1])]
                #
                # while(futures[0].result() is None or futures[1].result()
                # is None):
                #     time.sleep(0.1)
                #

            else:
                topic_name = "goal"
                client = self.node.create_client(self.classControl,
                                                 self.node_name + "/" +
                                                 topic_name)

                counter = 10
                while self.keepThreads and not client.wait_for_service(
                        timeout_sec=0.5) and counter > 0:
                    if counter % 2 == 0:
                        self.node.get_logger().info(
                            topic_name + ' service not available, '
                                         'waiting again...')
                    counter -= 1

                if counter <= 0 or not self.keepThreads:
                    self.node.get_logger().info(
                        topic_name + ' service not available, '
                                     'please try again...')
                    sys.exit()

                req = self.classControl.Request()
                req.goal_linearposition = position
                req.goal_angularposition = rotation

                print('moveGripper', "%.3f" % req.goal_linearposition,
                      "%.3f" % req.goal_angularposition)

                future = client.call_async(req)

                while future.result() is None:
                    time.sleep(0.1)

                print(future.result())

                self.node.destroy_client(client)

            sys.exit()
        except Exception as e:
            raise e

    #########################################################
    # Logic                                                 #
    #########################################################

    # Start threads if selected node is valid
    def connect_to_node(self):
        thread_id = threading.Thread(target=self.worker_id)
        thread_id.start()

        # Spinning thread
        spinner = threading.Thread(target=self.worker_spinner)
        spinner.start()

    # Latch onto component specific topics and services
    def latch_to_node(self):

        thread_specs = threading.Thread(target=self.worker_specs)
        thread_specs.start()

        thread_simulation = threading.Thread(target=self.worker_simulation)
        thread_simulation.start()

        thread_state = threading.Thread(target=self.worker_state)
        thread_state.start()

        # # TODO: finalize thread migration
        # threadSimulation = threading.Thread(target=self.workerSimulation)
        # threadSimulation.start()

    # Refreshes list of nodes on node selection combobox
    def list_nodes(self):
        try:

            # self.cleanup()
            nodes = self.node.get_node_names_and_namespaces()

            self.wSelNode.blockSignals(True)
            self.wSelNode.clear()
            self.wSelNode.addItem("")
            for node in nodes:
                if (not node[0] == self.node.get_name() and
                        node[0] not in self.node_list and
                        "hrim_" in node[0]):
                    self.node_list.append(node[0])
            self.wSelNode.addItems(self.node_list)
            self.wSelNode.update()
            self.wSelNode.blockSignals(False)
        except Exception as e:
            raise e

    # ROS2 related cleanup when losing connection to a node
    def cleanup(self):

        self.executor.shutdown()
        time.sleep(0.05)

        self.node.destroy_node()
        time.sleep(0.05)

        self.node = rclpy.create_node(
            'hrim_control_' + str(threading.get_ident()), context=self.context)

        self.executor = localExecutor(context=self.context)
        self.executor.add_node(self.node)

    def set_id_info(self, result):

        global IMPORT_TYPES

        if result.device_name == "":
            self.type = "Servo"
        else:
            pass

        if self.type == "Gripper":
            self.withAction = False
        if self.type == "Servo":
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

        self.latch_to_node()

        self.handle_id(result)

    # 3D model zip decompression into temporary files
    def decompress(self):
        # Get bytestring from message
        data = b''.join(self.model)

        # Check if tmp path exists, delete it if it does
        path = os.path.join(os.getcwd(), "tmp")
        if os.path.exists(path):
            shutil.rmtree(os.path.join(os.getcwd(), "tmp"))
        os.makedirs(path)

        # Write compressed file
        filename = "description.zip"
        out_file = open(os.path.join(path, filename),
                        "wb")  # open for [w]riting as [b]inary
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

        urdf_file = None
        for file in os.listdir(path):
            if file.endswith(".urdf"):
                urdf_file = os.path.join(os.getcwd(), path, file)

        tmp_file = open(urdf_file, "r")
        tmp_contents = tmp_file.read()
        tmp_file.close()

        os.remove(urdf_file)

        processed_urdf = re.sub(
            r'( *<mesh filename=")package://.*(/[^/]*.dae)(".*)',
            r'\g<1>file://' + os.path.join(os.getcwd(), path) + '\g<2>\g<3>',
            tmp_contents)
        processed_urdf = re.sub(r'(<joint.*type=")(.*)(">)',
                                r'\g<1>fixed\g<3>', processed_urdf)

        with open(os.path.join(path, "temporary.urdf"), "w") as myFile:
            myFile.write(processed_urdf)

        with open("config.rviz", "r") as myFile:
            base_rviz = myFile.read()

        processed_rviz = re.sub(r'%FIXED_FRAME%', self.model_frame, base_rviz)
        processed_rviz = re.sub(r'%URDF_FILE%',
                                os.path.join(path, "temporary.urdf"),
                                processed_rviz)

        with open(os.path.join("tmp", "temporary.rviz"), "w") as myFile:
            myFile.write(processed_rviz)

        os.environ["HRIM_TMP_URDF"] = os.path.join(path, "temporary.urdf")
        os.environ["HRIM_TMP_RVIZ"] = os.path.join(path, "temporary.rviz")

        # self.stateGroup.setTabEnabled(2, True)

        # self.rviz_proc = subprocess.Popen("rviz2 -d "+os.path.join("tmp",
        # "temporary.rviz"), shell = True)
        # self.rviz_proc = subprocess.Popen(
        # "ros2 launch my_dummy_robot_bringup trying.launch.py", shell = True)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = HrimGripperControl(app)
    sys.exit(app.exec_())
