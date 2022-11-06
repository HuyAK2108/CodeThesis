from turtle import speed
from PyQt5 import QtCore
from PyQt5.QtCore import QObject, QThread, pyqtSignal
from PyQt5.QtWidgets import *
from typing import *
from typedef import *
from motomini import Motomini
import sys, time, serial

device = Motomini()
class Robot(QThread):
    get_position = pyqtSignal(str, str, str, str, str, str,
                              str, str, str, str, str, str)
    
    def __init__(self, index = 0):
        super(Robot, self).__init__()
        self.index = index
        self.connection = device
        self.connection.connectMotomini(ip = "192.168.1.12", port = 10040)
        print("Robot thread start", self.index)

    def run(self):
        self.start_receive_pos()
        
    def start_receive_pos(self):
        self.timer_update_gui = QtCore.QTimer()
        self.timer_update_gui.setInterval(500)
        self.timer_update_gui.timeout.connect(self.updateGUI)
        self.timer_update_gui.start()
    
    def stop_receive_pos(self):
        self.timer_update_gui.stop()
        self.stop()

    def stop(self):
        """Sets waits for thread to finish"""
        print("Robot thread stop", self.index)
        self.wait()
        
    def updateGUI(self):
        device.getCartasianPos()
        time.sleep(0.0005)
        device.getPulsePos()
        time.sleep(0.0005)
        device.convertPos()
          
        txt_X     = (str (round (float(constVariable.CartesianPos[0]) / 1000 , 3 ) ) )
        txt_Y     = (str (round (float(constVariable.CartesianPos[1]) / 1000 , 3 ) ) )
        txt_Z     = (str (round (float(constVariable.CartesianPos[2]) / 1000 , 3 ) ) )
        txt_Roll  = (str (round (float(constVariable.CartesianPos[3]) / 10000, 4) ) )
        txt_Pitch = (str (round (float(constVariable.CartesianPos[4]) / 10000, 4) ) )
        txt_Yaw   = (str (round (float(constVariable.CartesianPos[5]) / 10000, 4) ) )
        
        txt_S     = (str (round (float(constVariable.PulsePos[0]) / constVariable.pulse_per_degree_S   ,3 ) ) )
        txt_L     = (str (round (float(constVariable.PulsePos[1]) / constVariable.pulse_per_degree_L   ,3 ) ) )
        txt_U     = (str (round (float(constVariable.PulsePos[2]) / constVariable.pulse_per_degree_U   ,3 ) ) )
        txt_R     = (str (round (float(constVariable.PulsePos[3]) / constVariable.pulse_per_degree_RBT ,3 ) ) )
        txt_B     = (str (round (float(constVariable.PulsePos[4]) / constVariable.pulse_per_degree_RBT ,3 ) ) )
        txt_T     = (str (round (float(constVariable.PulsePos[5]) / constVariable.pulse_per_degree_RBT ,3 ) ) )
        
        
        self.get_position.emit(txt_X, txt_Y, txt_Z, txt_Roll, txt_Pitch, txt_Yaw,
                               txt_S, txt_L, txt_U, txt_R, txt_B, txt_T)
         
class UART(QThread):
    get_speed = pyqtSignal(int)
    def __init__(self, index = 0, com = '', baudrate = 0):
        super(UART, self).__init__()
        self.index = index
        self.com = com
        self.baudrate = baudrate
        self.connection = device
        self.ready_to_read = False
        self.start_receive = 'S'
        self.stop_receive = 'T'
        print("UART start thread", self.index)
        
    def init_timer(self):
        self.ready_to_read = True
        self.ser = serial.Serial(self.com, self.baudrate, timeout = 2.5)
        self.ser.write(self.start_receive.encode())
        self.TIMER_UART = QtCore.QTimer()
        self.TIMER_UART.setInterval(500)
        self.TIMER_UART.timeout.connect(self.get_data)
        self.TIMER_UART.start()
    
    def stop_timer(self):
        self.ser.write(self.stop_receive.encode())
        self.TIMER_UART.stop()
        self.ser.close()
        
    def stop(self):
        """Sets waits for thread to finish"""
        print("UART stop thread", self.index)
            
        self.wait()
       
    def get_data(self):    
        bytetoread = []
        if self.ready_to_read == True:
            bytetoread = self.ser.inWaiting()
            if bytetoread > 0:
                RXData = self.ser.readline(bytetoread)
                # RXData = str(RXData, 'UTF-8')
                # RXData = RXData.split("\n")
                speed = RXData[0]
                self.get_speed.emit(speed)
           
                