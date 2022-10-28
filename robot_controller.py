from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QObject, QThread, pyqtSignal, pyqtSlot
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
        print("Robot thread start ", self.index)
    
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
        print("Robot thread stop ", self.index)
        self.wait()
        
    def updateGUI(self):
        device.getCartasianPos()
        time.sleep(0.0005)
        device.getPulsePos()
        time.sleep(0.0005)
        device.convertPos()
        print(constVariable.CartesianPos)
        print(constVariable.PulsePos)
          
        txt_X     = (str (round (float(constVariable.CartesianPos[0]) / 1000 , 3 ) ) )
        txt_Y     = (str (round (float(constVariable.CartesianPos[1]) / 1000 , 3 ) ) )
        txt_Z     = (str (round (float(constVariable.CartesianPos[2]) / 1000 , 3 ) ) )
        txt_Roll  = (str (round (float(constVariable.CartesianPos[3]) / 10000, 4) ) )
        txt_Pitch = (str (round (float(constVariable.CartesianPos[4]) / 10000, 4) ) )
        txt_Yaw   = (str (round (float(constVariable.CartesianPos[5]) / 10000, 4) ) )
        
        txt_S     = (str (round (float(constVariable.PulsePos[0]) / constVariable.pulse_per_degree_S   ,4 ) ) )
        txt_L     = (str (round (float(constVariable.PulsePos[1]) / constVariable.pulse_per_degree_L   ,4 ) ) )
        txt_U     = (str (round (float(constVariable.PulsePos[2]) / constVariable.pulse_per_degree_U   ,4 ) ) )
        txt_R     = (str (round (float(constVariable.PulsePos[3]) / constVariable.pulse_per_degree_RBT ,4 ) ) )
        txt_B     = (str (round (float(constVariable.PulsePos[4]) / constVariable.pulse_per_degree_RBT ,4 ) ) )
        txt_T     = (str (round (float(constVariable.PulsePos[5]) / constVariable.pulse_per_degree_RBT ,4 ) ) )
        
        
        self.get_position.emit(txt_X, txt_Y, txt_Z, txt_Roll, txt_Pitch, txt_Yaw,
                               txt_S, txt_L, txt_U, txt_R, txt_B, txt_T)
# class Manual(QThread):
#     def __init__(self, index = 0, n = 0, speed = 0):
#         super(Manual, self).__init__()
#         self.index = index
#         self.n = n
#         self.connection = device
        
#     def run(self):
#         print("1")
        
#     def decX(self):
#         pos: list = []
#         speed: int32 = self.uic.Speed_Slider.value() * 50
#         if self.uic.Cartesian_coor.isChecked() == True:
#             x_pos = constVariable.CartesianPos[0] - speed * 10
#             y_pos = constVariable.CartesianPos[1]
#             z_pos = constVariable.CartesianPos[2]
#             roll_pos = constVariable.CartesianPos[3]
#             pitch_pos = constVariable.CartesianPos[4]
#             yaw_pos = constVariable.CartesianPos[5]
#             pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
#             self.connection.moveCartasianPos(speed, pos)
#         else:
#             s_pos = constVariable.PulsePos[0] - speed * 10
#             l_pos = constVariable.PulsePos[1]
#             u_pos = constVariable.PulsePos[2]
#             r_pos = constVariable.PulsePos[3]
#             b_pos = constVariable.PulsePos[4]
#             t_pos = constVariable.PulsePos[5]
#             pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
#             self.connection.movePulsePos(speed, pos)

#     def incX(self):
#         pos: list = []
#         speed: int32 = self.uic.Speed_Slider.value() * 50
#         if self.uic.Cartesian_coor.isChecked() == True:
#             x_pos = constVariable.CartesianPos[0] + speed * 10
#             y_pos = constVariable.CartesianPos[1]
#             z_pos = constVariable.CartesianPos[2]
#             roll_pos = constVariable.CartesianPos[3]
#             pitch_pos = constVariable.CartesianPos[4]
#             yaw_pos = constVariable.CartesianPos[5]
#             pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
#             self.connection.moveCartasianPos(speed, pos)
#         else:
#             s_pos = constVariable.PulsePos[0] + speed * 10
#             l_pos = constVariable.PulsePos[1]
#             u_pos = constVariable.PulsePos[2]
#             r_pos = constVariable.PulsePos[3]
#             b_pos = constVariable.PulsePos[4]
#             t_pos = constVariable.PulsePos[5]
#             pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
#             self.connection.movePulsePos(speed, pos)

#     def decY(self):
#         pos: list = []
#         speed: int32 = self.uic.Speed_Slider.value() * 50
#         if self.uic.Cartesian_coor.isChecked() == True:
#             x_pos = constVariable.CartesianPos[0]
#             y_pos = constVariable.CartesianPos[1] - speed * 10
#             z_pos = constVariable.CartesianPos[2]
#             roll_pos = constVariable.CartesianPos[3]
#             pitch_pos = constVariable.CartesianPos[4]
#             yaw_pos = constVariable.CartesianPos[5]
#             pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
#             self.connection.moveCartasianPos(speed, pos)
#         else:
#             s_pos = constVariable.PulsePos[0]
#             l_pos = constVariable.PulsePos[1] - speed * 10
#             u_pos = constVariable.PulsePos[2]
#             r_pos = constVariable.PulsePos[3]
#             b_pos = constVariable.PulsePos[4]
#             t_pos = constVariable.PulsePos[5]
#             pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
#             self.connection.movePulsePos(speed, pos)

#     def incY(self):
#         pos: list = []
#         speed: int32 = self.uic.Speed_Slider.value() * 50
#         if self.uic.Cartesian_coor.isChecked() == True:
#             x_pos = constVariable.CartesianPos[0]
#             y_pos = constVariable.CartesianPos[1] + speed * 10
#             z_pos = constVariable.CartesianPos[2]
#             roll_pos = constVariable.CartesianPos[3]
#             pitch_pos = constVariable.CartesianPos[4]
#             yaw_pos = constVariable.CartesianPos[5]
#             pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
#             self.connection.moveCartasianPos(speed, pos)
#         else:
#             s_pos = constVariable.PulsePos[0]
#             l_pos = constVariable.PulsePos[1] + speed * 10
#             u_pos = constVariable.PulsePos[2]
#             r_pos = constVariable.PulsePos[3]
#             b_pos = constVariable.PulsePos[4]
#             t_pos = constVariable.PulsePos[5]
#             pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
#             self.connection.movePulsePos(speed, pos)

#     def decZ(self):
#         pos: list = []
#         speed: int32 = self.uic.Speed_Slider.value() * 50
#         if self.uic.Cartesian_coor.isChecked() == True:
#             x_pos = constVariable.CartesianPos[0]
#             y_pos = constVariable.CartesianPos[1]
#             z_pos = constVariable.CartesianPos[2] - speed * 10
#             roll_pos = constVariable.CartesianPos[3]
#             pitch_pos = constVariable.CartesianPos[4]
#             yaw_pos = constVariable.CartesianPos[5]
#             pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
#             self.connection.moveCartasianPos(speed, pos)
#         else:
#             s_pos = constVariable.PulsePos[0]
#             l_pos = constVariable.PulsePos[1]
#             u_pos = constVariable.PulsePos[2] - speed * 10
#             r_pos = constVariable.PulsePos[3]
#             b_pos = constVariable.PulsePos[4]
#             t_pos = constVariable.PulsePos[5]
#             pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
#             self.connection.movePulsePos(speed, pos)

#     def incZ(self):
#         pos: list = []
#         speed: int32 = self.uic.Speed_Slider.value() * 50
#         if self.uic.Cartesian_coor.isChecked() == True:
#             x_pos = constVariable.CartesianPos[0]
#             y_pos = constVariable.CartesianPos[1]
#             z_pos = constVariable.CartesianPos[2] + speed * 10
#             roll_pos = constVariable.CartesianPos[3]
#             pitch_pos = constVariable.CartesianPos[4]
#             yaw_pos = constVariable.CartesianPos[5]
#             pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
#             self.connection.moveCartasianPos(speed, pos)
#         else:
#             s_pos = constVariable.PulsePos[0]
#             l_pos = constVariable.PulsePos[1]
#             u_pos = constVariable.PulsePos[2] + speed * 10
#             r_pos = constVariable.PulsePos[3]
#             b_pos = constVariable.PulsePos[4]
#             t_pos = constVariable.PulsePos[5]
#             pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
#             self.connection.movePulsePos(speed, pos)

#     def decR(self):
#         pos: list = []
#         speed: int32 = self.uic.Speed_Slider.value() * 50
#         if self.uic.Cartesian_coor.isChecked() == True:
#             x_pos = constVariable.CartesianPos[0]
#             y_pos = constVariable.CartesianPos[1]
#             z_pos = constVariable.CartesianPos[2]
#             roll_pos = constVariable.CartesianPos[3] - speed * 10
#             pitch_pos = constVariable.CartesianPos[4]
#             yaw_pos = constVariable.CartesianPos[5]
#             pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
#             self.connection.moveCartasianPos(speed, pos)
#         else:
#             s_pos = constVariable.PulsePos[0]
#             l_pos = constVariable.PulsePos[1]
#             u_pos = constVariable.PulsePos[2]
#             r_pos = constVariable.PulsePos[3] - speed * 10
#             b_pos = constVariable.PulsePos[4]
#             t_pos = constVariable.PulsePos[5]
#             pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
#             self.connection.movePulsePos(speed, pos)

#     def incR(self):
#         pos: list = []
#         speed: int32 = self.uic.Speed_Slider.value() * 50
#         if self.uic.Cartesian_coor.isChecked() == True:
#             x_pos = constVariable.CartesianPos[0]
#             y_pos = constVariable.CartesianPos[1]
#             z_pos = constVariable.CartesianPos[2]
#             roll_pos = constVariable.CartesianPos[3] + speed * 10
#             pitch_pos = constVariable.CartesianPos[4]
#             yaw_pos = constVariable.CartesianPos[5]
#             pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
#             self.connection.moveCartasianPos(speed, pos)
#         else:
#             s_pos = constVariable.PulsePos[0]
#             l_pos = constVariable.PulsePos[1]
#             u_pos = constVariable.PulsePos[2]
#             r_pos = constVariable.PulsePos[3] + speed * 10
#             b_pos = constVariable.PulsePos[4]
#             t_pos = constVariable.PulsePos[5]
#             pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
#             self.connection.movePulsePos(speed, pos)

#     def decP(self):
#         pos: list = []
#         speed: int32 = self.uic.Speed_Slider.value() * 50
#         if self.uic.Cartesian_coor.isChecked() == True:
#             x_pos = constVariable.CartesianPos[0]
#             y_pos = constVariable.CartesianPos[1]
#             z_pos = constVariable.CartesianPos[2]
#             roll_pos = constVariable.CartesianPos[3]
#             pitch_pos = constVariable.CartesianPos[4] - speed * 10
#             yaw_pos = constVariable.CartesianPos[5]
#             pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
#             self.connection.moveCartasianPos(speed, pos)
#         else:
#             s_pos = constVariable.PulsePos[0]
#             l_pos = constVariable.PulsePos[1]
#             u_pos = constVariable.PulsePos[2]
#             r_pos = constVariable.PulsePos[3]
#             b_pos = constVariable.PulsePos[4] - speed * 10
#             t_pos = constVariable.PulsePos[5]
#             pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
#             self.connection.movePulsePos(speed, pos)

#     def incP(self):
#         pos: list = []
#         speed: int32 = self.uic.Speed_Slider.value() * 50
#         if self.uic.Cartesian_coor.isChecked() == True:
#             x_pos = constVariable.CartesianPos[0]
#             y_pos = constVariable.CartesianPos[1]
#             z_pos = constVariable.CartesianPos[2]
#             roll_pos = constVariable.CartesianPos[3]
#             pitch_pos = constVariable.CartesianPos[4] + speed * 10
#             yaw_pos = constVariable.CartesianPos[5]
#             pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
#             self.connection.moveCartasianPos(speed, pos)
#         else:
#             s_pos = constVariable.PulsePos[0]
#             l_pos = constVariable.PulsePos[1]
#             u_pos = constVariable.PulsePos[2]
#             r_pos = constVariable.PulsePos[3]
#             b_pos = constVariable.PulsePos[4] + speed * 10
#             t_pos = constVariable.PulsePos[5]
#             pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
#             self.connection.movePulsePos(speed, pos)

#     def decYaw(self):
#         pos: list = []
#         speed: int32 = self.uic.Speed_Slider.value() * 50
#         if self.uic.Cartesian_coor.isChecked() == True:
#             x_pos = constVariable.CartesianPos[0]
#             y_pos = constVariable.CartesianPos[1]
#             z_pos = constVariable.CartesianPos[2]
#             roll_pos = constVariable.CartesianPos[3]
#             pitch_pos = constVariable.CartesianPos[4]
#             yaw_pos = constVariable.CartesianPos[5] - speed * 10
#             pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
#             self.connection.moveCartasianPos(speed, pos)
#         else:
#             s_pos = constVariable.PulsePos[0]
#             l_pos = constVariable.PulsePos[1]
#             u_pos = constVariable.PulsePos[2]
#             r_pos = constVariable.PulsePos[3]
#             b_pos = constVariable.PulsePos[4]
#             t_pos = constVariable.PulsePos[5] - speed * 10
#             pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
#             self.connection.movePulsePos(speed, pos)

#     def incYaw(self):
#         pos: list = []
#         speed: int32 = self.uic.Speed_Slider.value() * 50
#         if self.uic.Cartesian_coor.isChecked() == True:
#             x_pos = constVariable.CartesianPos[0]
#             y_pos = constVariable.CartesianPos[1]
#             z_pos = constVariable.CartesianPos[2]
#             roll_pos = constVariable.CartesianPos[3]
#             pitch_pos = constVariable.CartesianPos[4]
#             yaw_pos = constVariable.CartesianPos[5] + speed * 10
#             pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
#             self.connection.moveCartasianPos(speed, pos)
#         else:
#             s_pos = constVariable.PulsePos[0]
#             l_pos = constVariable.PulsePos[1]
#             u_pos = constVariable.PulsePos[2]
#             r_pos = constVariable.PulsePos[3]
#             b_pos = constVariable.PulsePos[4]
#             t_pos = constVariable.PulsePos[5] + speed * 10
#             pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
#             self.connection.movePulsePos(speed, pos)    
        
        
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
                