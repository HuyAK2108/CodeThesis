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
                              str, str, str, str, str, str,
                              int, int)
    
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
        self.timer_update_gui.setInterval(300)
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
        value_byte_22 = device.getByte(22)
        value_byte_23 = device.getByte(23)
          
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
                               txt_S, txt_L, txt_U, txt_R, txt_B, txt_T,
                               value_byte_22, value_byte_23)
class Auto_system(QThread):
    timer_count = pyqtSignal(float) 
    
    def __init__(self, index = 0, robot_status = False):
        super(Auto_system, self).__init__()
        self.index = index
        # self.robot_connection = robot_status
        self.robot_connection = True
        self.count = 0
        self.run_flag = False
        # self.connection.connectMotomini(ip = "192.168.1.12", port = 10040)
        print("Auto thread start", self.index)
          
    def start_program(self):
        self.run_flag = True
        self.timer_auto = QtCore.QTimer()
        self.timer_auto.setInterval(100)
        self.timer_auto.timeout.connect(self.update_program)
        self.timer_auto.start()

    def stop_program(self):
        self.timer_auto.stop()
        self.stop()

    def stop(self):
        """Sets waits for thread to finish"""
        print("Auto thread stop", self.index)
        self.wait()
        
    def update_program(self):
        # print("name: " + flag.name + ", count: " + str(flag.flag_haohao))
        
        """ Method 1: Pick at one place
        """
        # if self.robot_connection == True:
        # # if Motomini.connect_status == True:
        #     x_pos       =  250  * 1000
        #     y_pos       = -100  * 1000
        #     z_pos       = -120  * 1000
        #     roll_pos    = -180  * 10000
        #     pitch_pos   =  0    * 10000
        #     yaw_pos     =  0    * 1000

        #     pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
        #     Motomini.writeVariablePos(121, pos)
            
        #     if flag.name == "Cung dinh":
        #         if flag.flag_cungdinh == 1:
        #             print("write byte 1")
        #             Motomini.writeByte(21,1)

        #     if flag.name == "Hao Hao":
        #         if flag.flag_haohao == 1:
        #             print("write byte 2")
        #             Motomini.writeByte(21,2)

        #     if flag.name == "Kokomi":
        #         if flag.flag_kokomi == 1:
        #             print("write byte 3")
        #             Motomini.writeByte(21,3)

        #     if flag.name == "Miliket":
        #         if flag.flag_miliket == 1:
        #             print("write byte 4")
        #             Motomini.writeByte(21,4)

        #     if flag.name == "Omachi":
        #         if flag.flag_omachi == 1:
        #             print("write byte 5")
        #             Motomini.writeByte(21,5)
                    
        if self.robot_connection == True and self.run_flag == True:
            if Byte.B022 == 1:
                self.count += 0.1
                print("sec: {:.2f}".format(self.count))
                if self.count > 1:
                    Byte.B023 = 1
                if Byte.B023 == 1 or self.count > 7:
                    self.run_flag = False 
                self.timer_count.emit(self.count)  
                 
        
        
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
           
                