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
    get_position = pyqtSignal(str, str, str, str, str, str)
    def __init__(self, index = 0):
        super(Robot, self).__init__()
        self.index = index
        print("Robot start thread ", self.index)
        # self.init_variable()
        # self.init_timer()
        # self.updateGUI()
        # self.ctrlServoCallback()
        # self.init_callback()
        
        self.connection = device

    def init_variable(self):
        self.com_connect_status = False
        self.read_data = False
        self.ser = serial.Serial()
    
    def init_timer(self):
        self.TIMER_UART = QtCore.QTimer()
        self.TIMER_UART.setInterval(500)
        # self.TIMER_UART.timeout.connect(self.get_data)
        # self.TIMER_UART.start()
        
        self.timer_update_gui = QtCore.QTimer()
        self.timer_update_gui.setInterval(500)
        self.timer_update_gui.timeout.connect(self.updateGUI)
        self.timer_update_gui.start()
    
    def checkConnect(func):
        def check(self):
            if device.checkConnectStatus() == False:
                print("Warning!!! NO connection with robot")
                return
            return func(self)

        return check
    
    def checkServo(func):
        def check(self):
            if device.checkServoStatus() == False:
                print("Warning!!! Servo is off")
                return
            return func(self)

        return check

    # def connectCallback(self):
    #     if device.checkConnectStatus() == False:
    #         device.connectMotomini(ip = "192.168.1.12", port = 10040)
    #         status = "DISCONNECT"
    #         # self.uic.btn_setconnnect.setText("DISCONNECT")
    #         # self.uic.btn_setconnnect.setStyleSheet("QPushButton {color: red;}")
    #         # self.timer_update_gui.start()
            
    #     else:
    #         status = "CONNECT"
    #         # self.uic.btn_setconnnect.setText("CONNECT")
    #         # self.uic.btn_setconnnect.setStyleSheet("QPushButton {color: green;}")
    #         self.timer_update_gui.stop()
    @checkConnect
    def ctrlServoCallback(self):
        if self.connection.checkServoStatus() == False:
            if self.connection.onServo() == 0:
                # gui.btn_servoON.setText("SERVO OFF")
                # gui.btn_servoON.setStyleSheet("QPushButton {color: red;}")
                # gui.lb_run_status.setText("ON")
                print("Servo is ON")
            else:
                print("Can not turn on servo")
        # else:
        #     if self.connection.offServo() == 0:
        #         # gui.btn_servoON.setText("SERVO ON") 
        #         # gui.btn_servoON.setStyleSheet("QPushButton {color: green;}")
        #         # gui.lb_run_status.setText("OFF")
        #         print("Servo is OFF")
        #     else:
        #         print("Can not turn off servo")
                
    def ctrlServoOff(self):
        if self.connection.checkServoStatus() == True:
            if self.connection.offServo() == 0:
                print("Servo is off")
            else:
                print("Can not turn off servo")
                
    def stop(self):
        """Sets waits for thread to finish"""
        self.wait()
                   
    def updateGUI(self):
        txt_X     = (str (round (float(constVariable.CartesianPos[0]) / 1000, 3 ) ) )
        txt_Y     = (str (round (float(constVariable.CartesianPos[1]) / 1000, 3 ) ) )
        txt_Z     = (str (round (float(constVariable.CartesianPos[2]) / 1000, 3 ) ) )
        txt_Roll  = (str (round (float(constVariable.CartesianPos[3]) / 10000, 4) ) )
        txt_Pitch = (str (round (float(constVariable.CartesianPos[4]) / 10000, 4) ) )
        txt_Yaw   = (str (round (float(constVariable.CartesianPos[5]) / 10000, 4) ) )
        self.get_position.emit(txt_X, txt_Y, txt_Z, txt_Roll, txt_Pitch, txt_Yaw)
#         gui.S_pos_text.setText(
#             str(
#                 round(
#                     float(constVariable.PulsePos[0]) / constVariable.pulse_per_degree_S,
#                     4,
#                 )
#             )
#         )
#         gui.L_pos_text.setText(
#             str(
#                 round(
#                     float(constVariable.PulsePos[1]) / constVariable.pulse_per_degree_L,
#                     4,
#                 )
#             )
#         )
#         gui.U_pos_text.setText(
#             str(
#                 round(
#                     float(constVariable.PulsePos[2]) / constVariable.pulse_per_degree_U,
#                     4,
#                 )
#             )
#         )
#         gui.R_pos_text.setText(
#             str(
#                 round(
#                     float(constVariable.PulsePos[3])
#                     / constVariable.pulse_per_degree_RBT,
#                     4,
#                 )
#             )
#         )
#         gui.B_pos_text.setText(
#             str(
#                 round(
#                     float(constVariable.PulsePos[4])
#                     / constVariable.pulse_per_degree_RBT,
#                     4,
#                 )
#             )
#         )
#         gui.T_pos_text.setText(
#             str(
#                 round(
#                     float(constVariable.PulsePos[5])
#                     / constVariable.pulse_per_degree_RBT,
#                     4,
#                 )
#             )
#         )
# # 
#     # 
#     def get_data(self):
        
        
        