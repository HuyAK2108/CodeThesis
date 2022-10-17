from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QObject, QThread, pyqtSignal
from PyQt5.QtWidgets import *
from typing import *
from typedef import *
from motomini import Motomini
import sys, time, serial
from gui import Ui_MainWindow

gui = Ui_MainWindow()
device = Motomini()
class GetPosition(QThread):
    finished = pyqtSignal()
    # progress = pyqtSignal()

    def run(self, device):
        device.getCartasianPos()
        time.sleep(0.005)
        device.getPulsePos()
        time.sleep(0.005)
        device.convertPos()
        # self.progress.emit()
        print(constVariable.CartesianPos)
        print(constVariable.PulsePos)
        self.finished.emit()
class Robot(QThread):
    def __init__(self, index = 0):
        super(Robot, self).__init__()
        self.init_variable()
        self.init_timer()
        self.init_callback()
        self.index = index
        print("Robot start thread ", self.index)
        
        self.connection = device
    
    def init_variable(self):
        self.com_connect_status = False
        self.read_data = False
        self.ser = serial.Serial()
    
    def init_timer(self):
        self.TIMER_UART = QtCore.QTimer()
        self.TIMER_UART.setInterval(500)
        # self.TIMER_UART.timeout.connect(self.get_data)
        self.TIMER_UART.start()
        
        self.timer_update_gui = QtCore.QTimer()
        self.timer_update_gui.setInterval(500)
        self.timer_update_gui.timeout.connect(self.updateGUI)
        
    def init_callback(self):
        print("callback")
    
    def checkConnect(func):
        def check(self):
            if self.connection.checkConnectStatus() == False:
                self.error_msg("Warning ! No connection to robot")
                return
            return func(self)
        return check
    
    def checkServo(func):
        def check(self):
            if self.connection.checkServoStatus() == False:
                self.error_msg("Warning!!! Servo is off")
                return
            return func(self)

        return check
    
    def connectCallback(self):
        if self.connection.checkConnectStatus() == False:
            self.connection.connectMotomini(ip = gui.text_IP.toPlainText(), port = int(gui.text_Port.toPlainText()))
            gui.btn_setconnnect.setText("DISCONNECT")
            gui.btn_setconnnect.setStyleSheet("QPushButton {color: red;}")
            self.timer_update_gui.start()
            
        else:
            gui.btn_setconnnect.setText("CONNECT")
            gui.btn_setconnnect.setStyleSheet("QPushButton {color: green;}")
            self.timer_update_gui.stop()
    
    @checkConnect
    def ctrlServoCallback(self):
        if self.connection.checkConnectStatus() == False:
            if self.connection.onServo() == 0:
                gui.btn_servoON.setText("SERVO OFF")
                gui.btn_servoON.setStyleSheet("QPushButton {color: red;}")
                gui.lb_run_status.setText("ON")
                print("Servo is ON")
            else:
                print("Servo is OFF")
        else:
            if self.connection.offServo() == 0:
                gui.btn_servoON.setText("SERVO ON") 
                gui.btn_servoON.setStyleSheet("QPushButton {color: green;}")
                gui.lb_run_status.setText("OFF")
                print("Servo is OFF")
            else:
                print("Cannot turn off servo")
                
                
    def updateGUI(self):
        gui.txt_X.setText(
            str(round(float(constVariable.CartesianPos[0]) / 1000, 3))
        )
        gui.txt_Y.setText(
            str(round(float(constVariable.CartesianPos[1]) / 1000, 3))
        )
        gui.txt_Z.setText(
            str(round(float(constVariable.CartesianPos[2]) / 1000, 3))
        )
        gui.txt_Roll.setText(
            str(round(float(constVariable.CartesianPos[3]) / 10000, 4))
        )
        gui.txt_Pitch.setText(
            str(round(float(constVariable.CartesianPos[4]) / 10000, 4))
        )
        gui.txt_Yaw.setText(
            str(round(float(constVariable.CartesianPos[5]) / 10000, 4))
        )

        # gui.S_pos_text.setText(
            # str(
                # round(
                    # float(constVariable.PulsePos[0]) / constVariable.pulse_per_degree_S,
                    # 4,
                # )
            # )
        # )
        # gui.L_pos_text.setText(
            # str(
                # round(
                    # float(constVariable.PulsePos[1]) / constVariable.pulse_per_degree_L,
                    # 4,
                # )
            # )
        # )
        # gui.U_pos_text.setText(
            # str(
                # round(
                    # float(constVariable.PulsePos[2]) / constVariable.pulse_per_degree_U,
                    # 4,
                # )
            # )
        # )
        # gui.R_pos_text.setText(
            # str(
                # round(
                    # float(constVariable.PulsePos[3])
                    # / constVariable.pulse_per_degree_RBT,
                    # 4,
                # )
            # )
        # )
        # gui.B_pos_text.setText(
            # str(
                # round(
                    # float(constVariable.PulsePos[4])
                    # / constVariable.pulse_per_degree_RBT,
                    # 4,
                # )
            # )
        # )
        # gui.T_pos_text.setText(
            # str(
                # round(
                    # float(constVariable.PulsePos[5])
                    # / constVariable.pulse_per_degree_RBT,
                    # 4,
                # )
            # )
        # )

    
    # def get_data(self):
        
        
        