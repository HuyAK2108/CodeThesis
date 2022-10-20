from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QObject, QThread, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import *
from typing import *
from typedef import *
from motomini import Motomini
import sys, time, serial

device = Motomini()
class GetPosition():
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
        self.connection = device
        print("Robot start thread ", self.index)
        
    def start_receive_pos(self):
        self.timer_update_gui = QtCore.QTimer()
        self.timer_update_gui.setInterval(500)
        self.timer_update_gui.timeout.connect(self.updateGUI)
        self.timer_update_gui.start()
    
    def stop_receive_pos(self):
        self.timer_update_gui.stop()

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

class UART(QThread):
    get_speed = pyqtSignal(float)
    def __init__(self, index = 0, com = '', baudrate = 0):
        super(UART, self).__init__()
        self.index = index
        self.com = com
        self.baudrate = baudrate
        self.connection = device
        print("UART start thread ", self.index) 
        print(self.com)
        print(self.baudrate)
        self.init_timer()
        
    def init_timer(self):
        self.ready_to_read = True
        self.ser = serial.Serial(self.com, self.baudrate, timeout = 2.5)
        self.TIMER_UART = QtCore.QTimer()
        self.TIMER_UART.setInterval(500)
        self.TIMER_UART.timeout.connect(self.get_data)
        self.TIMER_UART.start()
    
    def stop_timer(self):
        self.TIMER_UART.stop()
        self.ser.close()
        
    def stop(self):
        """Sets waits for thread to finish"""    
        self.wait()
       
    def get_data(self):    
        bytetoread = []
        if self.ready_to_read == True:
            bytetoread = self.ser.inWaiting()
            if bytetoread > 0:
                RXData = self.ser.readline(bytetoread)
                self.ready_to_read = False
                RXData = str(RXData, "UTF-8")
                RXData = RXData.split("\n")
                speed = RXData[0]
                self.get_speed.emit(speed)
                self.ready_to_read = True
                