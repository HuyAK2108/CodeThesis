import cv2, serial
import numpy as np
import serial.tools.list_ports
from PyQt5 import QtGui
from PyQt5.QtWidgets import QMainWindow, QSlider
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import  pyqtSlot, Qt, pyqtSignal, QTimer, QObject, QThread
from gui import Ui_MainWindow
from module import VideoThread
from robot_controller import Robot, UART
from motomini import Motomini
from typedef import *

device = Motomini()
class MainWindow (QMainWindow):
    def __init__(self):
        super().__init__()
        self.uic = Ui_MainWindow()
        self.uic.setupUi(self)
        self.init_variables()
        self.init_timer()
        self.init_button()
    
    def init_timer(self):
        self.timer_move_pos = QTimer()
        self.timer_move_pos.setInterval(300)
        self.timer_move_pos.timeout.connect(self.moveRobot)
        
    def init_variables(self):        
        # Init Thread
        self.thread = {}
        # Thread Status
        self.status_thread_1 = False
        self.status_thread_2 = False
        self.status_thread_3 = False
        
        # Init Button Status
        self.uic.btn_servoON.setEnabled(False)
        self.uic.btn_home_pos.setEnabled(False)
        self.uic.btn_start_job.setEnabled(False)
        self.uic.btn_stop_job.setEnabled(False)
        self.uic.btn_offCAM.setEnabled(False)
        self.uic.btn_closeuart.setEnabled(False)
        # self.uic.btn_setconnnect.setStyleSheet("QPushButton {color: green;}")
        # self.uic.btn_onCAM.setStyleSheet("QPushButton {color: green;}")
        # self.uic.btn_setuart.setStyleSheet("QPushButton {color: green;}")
        
        
        # Connection Status
        self.job_status = False
        self.com_connect_status = False
        
        # Call Motomini class
        self.connection = device
        
        # Get available serial ports
        self.comlist = serial.tools.list_ports.comports()
        self.connected = []
        for element in self.comlist:
            self.connected.append(element.device)
        self.uic.COM.addItems(self.connected)        
    
    def init_button(self):
        # Button ON - OFF camera
        self.uic.btn_onCAM.clicked.connect(self.START_CAPTURE_VIDEO)
        self.uic.btn_offCAM.clicked.connect(self.STOP_CAPTURE_VIDEO)
        # Button OPEN - CLOSE uart
        self.uic.btn_setuart.clicked.connect(self.CONNECT_SERIAL)
        self.uic.btn_closeuart.clicked.connect(self.DISCONNECT_SERIAL)
        # Button CONNECT - DISCONNECT robot
        self.uic.btn_setconnnect.clicked.connect(self.CONNECT_ROBOT)
        # Button ON - OFF servo
        self.uic.btn_servoON.clicked.connect(self.SERVO_ON)
        self.uic.btn_home_pos.clicked.connect(self.HOME_POSITION)
        # Button LOAD - START - STOP job
        self.uic.btn_load_job.clicked.connect(self.LOAD_JOB)
        self.uic.btn_start_job.clicked.connect(self.START_JOB)
        self.uic.btn_stop_job.clicked.connect(self.STOP_JOB)
        # Button RUN
        self.uic.move_butt.clicked.connect(self.RUN_ROBOT)
        # Press button jogging
        self.uic.DecX_butt.pressed.connect(lambda: self.btnJogPressCallback(1))
        self.uic.IncX_butt.pressed.connect(lambda: self.btnJogPressCallback(2))
        self.uic.DecY_butt.pressed.connect(lambda: self.btnJogPressCallback(3))
        self.uic.IncY_butt.pressed.connect(lambda: self.btnJogPressCallback(4))
        self.uic.DecZ_butt.pressed.connect(lambda: self.btnJogPressCallback(5))
        self.uic.IncZ_butt.pressed.connect(lambda: self.btnJogPressCallback(6))
        self.uic.DecR_butt.pressed.connect(lambda: self.btnJogPressCallback(7))
        self.uic.IncR_butt.pressed.connect(lambda: self.btnJogPressCallback(8))
        self.uic.DecP_butt.pressed.connect(lambda: self.btnJogPressCallback(9))
        self.uic.IncP_butt.pressed.connect(lambda: self.btnJogPressCallback(10))
        self.uic.DecYaw_butt.pressed.connect(lambda: self.btnJogPressCallback(11))
        self.uic.IncYaw_butt.pressed.connect(lambda: self.btnJogPressCallback(12))
        # Release button jogging
        self.uic.DecX_butt.released.connect(self.btnJogReleaseCallback)
        self.uic.IncX_butt.released.connect(self.btnJogReleaseCallback)
        self.uic.DecY_butt.released.connect(self.btnJogReleaseCallback)
        self.uic.IncY_butt.released.connect(self.btnJogReleaseCallback)
        self.uic.DecZ_butt.released.connect(self.btnJogReleaseCallback)
        self.uic.IncZ_butt.released.connect(self.btnJogReleaseCallback)
        self.uic.DecR_butt.released.connect(self.btnJogReleaseCallback)
        self.uic.IncR_butt.released.connect(self.btnJogReleaseCallback)
        self.uic.DecP_butt.released.connect(self.btnJogReleaseCallback)
        self.uic.IncP_butt.released.connect(self.btnJogReleaseCallback)
        self.uic.DecYaw_butt.released.connect(self.btnJogReleaseCallback)
        self.uic.IncYaw_butt.released.connect(self.btnJogReleaseCallback)
        # Button Coordinate
        self.uic.Cartesian_coor.clicked.connect(
            lambda: self.coorRadBtnCallback(self.uic.Cartesian_coor.isChecked()))
        self.uic.pulse_coor.clicked.connect(
            lambda: self.coorRadBtnCallback(self.uic.Cartesian_coor.isChecked()))   
        self.uic.Speed_Slider.valueChanged.connect(
            lambda: self.Speed_slider_change_callback(self.uic.Speed_Slider))
            
    def moveRobot(self):
        if self.moveSel == 1:
            self.decX()
        elif self.moveSel == 2:
            self.incX()
        elif self.moveSel == 3:
            self.decY()
        elif self.moveSel == 4:
            self.incY()
        elif self.moveSel == 5:
            self.decZ()
        elif self.moveSel == 6:
            self.incZ()
        elif self.moveSel == 7:
            self.decR()
        elif self.moveSel == 8:
            self.incR()
        elif self.moveSel == 9:
            self.decP()
        elif self.moveSel == 10:
            self.incP()
        elif self.moveSel == 11:
            self.decYaw()
        elif self.moveSel == 12:
            self.incYaw() 
            
    def START_CAPTURE_VIDEO(self):
        """Start video object detection and turn on camera
        """
        self.uic.btn_onCAM.setEnabled(False)
        self.uic.btn_offCAM.setEnabled(True)
        self.uic.btn_onCAM.setStyleSheet("QPushButton {color: green;}")
        self.uic.btn_offCAM.setStyleSheet("QPushButton {color: black;}")
        
        self.status_thread_1 = True
        # create the video capture thread
        self.thread[1] = VideoThread(index=1)
        # connect its signal to the show_info slot to display object name
        self.thread[1].signal.connect(self.show_info)
        self.thread[1].number.connect(self.show_number)
        # connect its signal to the update_image slot to display webcam
        self.thread[1].change_pixmap_signal.connect(self.update_image)
        # start the thread
        self.thread[1].start()
        
    def STOP_CAPTURE_VIDEO(self):
        """Stop capture video and turn off camera
        """
        self.uic.btn_onCAM.setEnabled(True)
        self.uic.btn_offCAM.setEnabled(False)
        self.uic.btn_onCAM.setStyleSheet("QPushButton {color: black;}")
        self.uic.btn_offCAM.setStyleSheet("QPushButton {color: red;}")
        self.thread[1].stop()
    
    def closeEvent(self, event):
        """Close Window
        """
        if self.status_thread_1 == True:
            self.thread[1].stop()
        if self.status_thread_2 == True:
            self.thread[2].stop()
        if self.status_thread_3 == True:
            self.thread[3].stop()
        event.accept()
        
    def CONNECT_SERIAL(self):
        """Open UART to send/receive data
        """
        self.com = self.uic.COM.currentText()
        self.baudrate = int(self.uic.Baudrate.currentText())
        if self.com_connect_status == False:
            print("UART connected")
            self.status_thread_3 = True
            self.com_connect_status = True
            self.uic.btn_closeuart.setEnabled(True)
            self.uic.btn_setuart.setEnabled(False)
            self.thread[3] = UART(index = 3, com = self.com, baudrate = self.baudrate)
            self.thread[3].init_timer()
            self.thread[3].get_speed.connect(self.show_speed)
        
    def DISCONNECT_SERIAL(self):
        """Close UART
        """
        self.com = self.uic.COM.currentText()   
        self.uic.text_speed.setText('0')     
        self.uic.btn_closeuart.setEnabled(False)
        self.uic.btn_setuart.setEnabled(True)
        self.com_connect_status = False
        self.thread[3].stop_timer()
        print("Disconnected to",self.com)    
    
    def checkConnect(func):
        def check(self):
            if self.connection.checkConnectStatus() == False:
                print("Warning!!! NO connection with robot")
                return
            return func(self)
        return check

    def checkServo(func):
        def check(self):
            if self.connection.checkServoStatus() == False:
                self.print("Warning!!! Servo is off")
                return
            return func(self)
        return check
    
    @checkConnect
    @checkServo
    def HOME_POSITION(self):
        self.connection.homeServo()
           
    @checkConnect
    def SERVO_ON(self):
        if self.connection.checkServoStatus() == False:
            if self.connection.onServo() == 0:       
                self.uic.btn_servoON.setText("SERVO OFF")
                self.uic.btn_servoON.setStyleSheet("QPushButton {color: red;}")
                self.uic.lb_run_status.setText("Robot Status: ON")
                self.status_thread_2 = True
                self.thread[2] = Robot(index = 2)
                self.thread[2].start_receive_pos()
                self.thread[2].get_position.connect(self.show_position)
            else:
                print("Can not on servo")
        else:
            if self.connection.offServo() == 0:
                self.uic.btn_servoON.setText("SERVO ON")
                self.uic.btn_servoON.setStyleSheet("QPushButton {color: green;}")
                self.uic.lb_run_status.setText("Robot Status: OFF")
                self.thread[2].stop_receive_pos()
            else:
                print("Can not off servo")
        
    def CONNECT_ROBOT(self):        
        if self.connection.checkConnectStatus() == False:
            # ip = self.uic.text_IP.text()
            # port = int(self.uic.text_Port.text())
            # self.connection.connectMotomini(ip = ip, port = port)
            self.uic.btn_servoON.setEnabled(True)
            self.uic.btn_home_pos.setEnabled(True)
            self.connection.connectMotomini(ip = "192.168.1.12", port = 10040)
            self.uic.btn_setconnnect.setText("DISCONNECT")
            self.uic.btn_setconnnect.setStyleSheet("QPushButton {color: red;}")
            print("Connected to Robot")
        else:
            self.connection.disconnectMotomini()
            self.uic.btn_setconnnect.setText("CONNECT")
            self.uic.btn_setconnnect.setStyleSheet("QPushButton {color: green;}")
            print("Disconnected to Robot")
    
    @checkConnect
    @checkServo
    def LOAD_JOB(self):
        self.uic.btn_load_job.setEnabled(False)
        self.uic.btn_stop_job.setEnabled(True)
        self.uic.btn_start_job.setEnabled(True)
        self.uic.btn_load_job.setStyleSheet("QPushButton {color: green;}")
        job = self.uic.text_JOB.text()
        line = 1
        # Start current Job at line 1
        self.connection.selectJob(job_name= job, line_no= line)
        self.job_status = True
                
    @checkConnect
    @checkServo
    def START_JOB(self):
        if self.job_status == True:
            self.connection.startJob()
        else:
            print("Can not load job")
            
    @checkConnect
    @checkServo
    def STOP_JOB(self):
        self.job_status = False
        self.uic.btn_load_job.setEnabled(True)
        self.uic.btn_stop_job.setEnabled(False)
        self.uic.btn_start_job.setEnabled(False)
        self.uic.btn_load_job.setStyleSheet("QPushButton {color: black;}")
        self.uic.btn_stop_job.setStyleSheet("QPushButton {color: red;}")
    
    # MANUAL
    def btnJogReleaseCallback(self):
        self.timer_move_pos.stop()
        self.moveSel = 0
    
    def btnJogPressCallback(self, n):
        if self.connection.checkConnectStatus() == True:
            if self.connection.checkServoStatus() == True:
                self.timer_move_pos.start()
                self.moveSel = n
            else:
                print("Warning!!! Servo is off")
        else:
            print("Warning!!! NO connection with robot")    
    
    def Speed_slider_change_callback(self, slider: QSlider):
        self.uic.progressBar.setValue(slider.value())
    
    # Run by clicking RUN button
    @checkConnect
    @checkServo
    def RUN_ROBOT(self):
        pos: list = []
        speed: int32 = int(self.uic.text_speed_2.text()) * 100
        if self.uic.Cartesian_coor.isChecked() == True:
            x_pos       = int( float (self.uic.S_move_text.toPlainText() ) * 1000)
            y_pos       = int( float (self.uic.L_move_text.toPlainText() ) * 1000)
            z_pos       = int( float (self.uic.U_move_text.toPlainText() ) * 1000)
            roll_pos    = int( float (self.uic.R_move_text.toPlainText() ) * 10000)
            pitch_pos   = int( float (self.uic.B_move_text.toPlainText() ) * 10000)
            yaw_pos     = int( float (self.uic.T_move_text.toPlainText() ) * 10000)
            
            pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
            
            self.connection.moveCartasianPos(speed, pos)
            
        else:
            s_pos = int( float (self.uic.S_move_text.toPlainText() ) * constVariable.pulse_per_degree_S)
            l_pos = int( float (self.uic.L_move_text.toPlainText() ) * constVariable.pulse_per_degree_L)
            u_pos = int( float (self.uic.U_move_text.toPlainText() ) * constVariable.pulse_per_degree_U)
            r_pos = int( float (self.uic.R_move_text.toPlainText() ) * constVariable.pulse_per_degree_RBT)
            b_pos = int( float (self.uic.B_move_text.toPlainText() ) * constVariable.pulse_per_degree_RBT)
            t_pos = int( float (self.uic.T_move_text.toPlainText() ) * constVariable.pulse_per_degree_RBT)
            
            pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
            
            self.connection.movePulsePos(speed, pos)
    
    # Run by pressing 
    def decX(self):
        pos: list = []
        speed: int32 = self.uic.Speed_Slider.value() * 50
        if self.uic.Cartesian_coor.isChecked() == True:
            x_pos = constVariable.CartesianPos[0] - speed * 10
            y_pos = constVariable.CartesianPos[1]
            z_pos = constVariable.CartesianPos[2]
            roll_pos = constVariable.CartesianPos[3]
            pitch_pos = constVariable.CartesianPos[4]
            yaw_pos = constVariable.CartesianPos[5]
            pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
            self.connection.moveCartasianPos(speed, pos)
        else:
            s_pos = constVariable.PulsePos[0] - speed * 10
            l_pos = constVariable.PulsePos[1]
            u_pos = constVariable.PulsePos[2]
            r_pos = constVariable.PulsePos[3]
            b_pos = constVariable.PulsePos[4]
            t_pos = constVariable.PulsePos[5]
            pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
            self.connection.movePulsePos(speed, pos)

    def incX(self):
        pos: list = []
        speed: int32 = self.uic.Speed_Slider.value() * 50
        if self.uic.Cartesian_coor.isChecked() == True:
            x_pos = constVariable.CartesianPos[0] + speed * 10
            y_pos = constVariable.CartesianPos[1]
            z_pos = constVariable.CartesianPos[2]
            roll_pos = constVariable.CartesianPos[3]
            pitch_pos = constVariable.CartesianPos[4]
            yaw_pos = constVariable.CartesianPos[5]
            pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
            self.connection.moveCartasianPos(speed, pos)
        else:
            s_pos = constVariable.PulsePos[0] + speed * 10
            l_pos = constVariable.PulsePos[1]
            u_pos = constVariable.PulsePos[2]
            r_pos = constVariable.PulsePos[3]
            b_pos = constVariable.PulsePos[4]
            t_pos = constVariable.PulsePos[5]
            pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
            self.connection.movePulsePos(speed, pos)

    def decY(self):
        pos: list = []
        speed: int32 = self.uic.Speed_Slider.value() * 50
        if self.uic.Cartesian_coor.isChecked() == True:
            x_pos = constVariable.CartesianPos[0]
            y_pos = constVariable.CartesianPos[1] - speed * 10
            z_pos = constVariable.CartesianPos[2]
            roll_pos = constVariable.CartesianPos[3]
            pitch_pos = constVariable.CartesianPos[4]
            yaw_pos = constVariable.CartesianPos[5]
            pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
            self.connection.moveCartasianPos(speed, pos)
        else:
            s_pos = constVariable.PulsePos[0]
            l_pos = constVariable.PulsePos[1] - speed * 10
            u_pos = constVariable.PulsePos[2]
            r_pos = constVariable.PulsePos[3]
            b_pos = constVariable.PulsePos[4]
            t_pos = constVariable.PulsePos[5]
            pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
            self.connection.movePulsePos(speed, pos)

    def incY(self):
        pos: list = []
        speed: int32 = self.uic.Speed_Slider.value() * 50
        if self.uic.Cartesian_coor.isChecked() == True:
            x_pos = constVariable.CartesianPos[0]
            y_pos = constVariable.CartesianPos[1] + speed * 10
            z_pos = constVariable.CartesianPos[2]
            roll_pos = constVariable.CartesianPos[3]
            pitch_pos = constVariable.CartesianPos[4]
            yaw_pos = constVariable.CartesianPos[5]
            pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
            self.connection.moveCartasianPos(speed, pos)
        else:
            s_pos = constVariable.PulsePos[0]
            l_pos = constVariable.PulsePos[1] + speed * 10
            u_pos = constVariable.PulsePos[2]
            r_pos = constVariable.PulsePos[3]
            b_pos = constVariable.PulsePos[4]
            t_pos = constVariable.PulsePos[5]
            pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
            self.connection.movePulsePos(speed, pos)

    def decZ(self):
        pos: list = []
        speed: int32 = self.uic.Speed_Slider.value() * 50
        if self.uic.Cartesian_coor.isChecked() == True:
            x_pos = constVariable.CartesianPos[0]
            y_pos = constVariable.CartesianPos[1]
            z_pos = constVariable.CartesianPos[2] - speed * 10
            roll_pos = constVariable.CartesianPos[3]
            pitch_pos = constVariable.CartesianPos[4]
            yaw_pos = constVariable.CartesianPos[5]
            pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
            self.connection.moveCartasianPos(speed, pos)
        else:
            s_pos = constVariable.PulsePos[0]
            l_pos = constVariable.PulsePos[1]
            u_pos = constVariable.PulsePos[2] - speed * 10
            r_pos = constVariable.PulsePos[3]
            b_pos = constVariable.PulsePos[4]
            t_pos = constVariable.PulsePos[5]
            pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
            self.connection.movePulsePos(speed, pos)

    def incZ(self):
        pos: list = []
        speed: int32 = self.uic.Speed_Slider.value() * 50
        if self.uic.Cartesian_coor.isChecked() == True:
            x_pos = constVariable.CartesianPos[0]
            y_pos = constVariable.CartesianPos[1]
            z_pos = constVariable.CartesianPos[2] + speed * 10
            roll_pos = constVariable.CartesianPos[3]
            pitch_pos = constVariable.CartesianPos[4]
            yaw_pos = constVariable.CartesianPos[5]
            pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
            self.connection.moveCartasianPos(speed, pos)
        else:
            s_pos = constVariable.PulsePos[0]
            l_pos = constVariable.PulsePos[1]
            u_pos = constVariable.PulsePos[2] + speed * 10
            r_pos = constVariable.PulsePos[3]
            b_pos = constVariable.PulsePos[4]
            t_pos = constVariable.PulsePos[5]
            pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
            self.connection.movePulsePos(speed, pos)

    def decR(self):
        pos: list = []
        speed: int32 = self.uic.Speed_Slider.value() * 50
        if self.uic.Cartesian_coor.isChecked() == True:
            x_pos = constVariable.CartesianPos[0]
            y_pos = constVariable.CartesianPos[1]
            z_pos = constVariable.CartesianPos[2]
            roll_pos = constVariable.CartesianPos[3] - speed * 10
            pitch_pos = constVariable.CartesianPos[4]
            yaw_pos = constVariable.CartesianPos[5]
            pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
            self.connection.moveCartasianPos(speed, pos)
        else:
            s_pos = constVariable.PulsePos[0]
            l_pos = constVariable.PulsePos[1]
            u_pos = constVariable.PulsePos[2]
            r_pos = constVariable.PulsePos[3] - speed * 10
            b_pos = constVariable.PulsePos[4]
            t_pos = constVariable.PulsePos[5]
            pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
            self.connection.movePulsePos(speed, pos)

    def incR(self):
        pos: list = []
        speed: int32 = self.uic.Speed_Slider.value() * 50
        if self.uic.Cartesian_coor.isChecked() == True:
            x_pos = constVariable.CartesianPos[0]
            y_pos = constVariable.CartesianPos[1]
            z_pos = constVariable.CartesianPos[2]
            roll_pos = constVariable.CartesianPos[3] + speed * 10
            pitch_pos = constVariable.CartesianPos[4]
            yaw_pos = constVariable.CartesianPos[5]
            pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
            self.connection.moveCartasianPos(speed, pos)
        else:
            s_pos = constVariable.PulsePos[0]
            l_pos = constVariable.PulsePos[1]
            u_pos = constVariable.PulsePos[2]
            r_pos = constVariable.PulsePos[3] + speed * 10
            b_pos = constVariable.PulsePos[4]
            t_pos = constVariable.PulsePos[5]
            pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
            self.connection.movePulsePos(speed, pos)

    def decP(self):
        pos: list = []
        speed: int32 = self.uic.Speed_Slider.value() * 50
        if self.uic.Cartesian_coor.isChecked() == True:
            x_pos = constVariable.CartesianPos[0]
            y_pos = constVariable.CartesianPos[1]
            z_pos = constVariable.CartesianPos[2]
            roll_pos = constVariable.CartesianPos[3]
            pitch_pos = constVariable.CartesianPos[4] - speed * 10
            yaw_pos = constVariable.CartesianPos[5]
            pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
            self.connection.moveCartasianPos(speed, pos)
        else:
            s_pos = constVariable.PulsePos[0]
            l_pos = constVariable.PulsePos[1]
            u_pos = constVariable.PulsePos[2]
            r_pos = constVariable.PulsePos[3]
            b_pos = constVariable.PulsePos[4] - speed * 10
            t_pos = constVariable.PulsePos[5]
            pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
            self.connection.movePulsePos(speed, pos)

    def incP(self):
        pos: list = []
        speed: int32 = self.uic.Speed_Slider.value() * 50
        if self.uic.Cartesian_coor.isChecked() == True:
            x_pos = constVariable.CartesianPos[0]
            y_pos = constVariable.CartesianPos[1]
            z_pos = constVariable.CartesianPos[2]
            roll_pos = constVariable.CartesianPos[3]
            pitch_pos = constVariable.CartesianPos[4] + speed * 10
            yaw_pos = constVariable.CartesianPos[5]
            pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
            self.connection.moveCartasianPos(speed, pos)
        else:
            s_pos = constVariable.PulsePos[0]
            l_pos = constVariable.PulsePos[1]
            u_pos = constVariable.PulsePos[2]
            r_pos = constVariable.PulsePos[3]
            b_pos = constVariable.PulsePos[4] + speed * 10
            t_pos = constVariable.PulsePos[5]
            pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
            self.connection.movePulsePos(speed, pos)

    def decYaw(self):
        pos: list = []
        speed: int32 = self.uic.Speed_Slider.value() * 50
        if self.uic.Cartesian_coor.isChecked() == True:
            x_pos = constVariable.CartesianPos[0]
            y_pos = constVariable.CartesianPos[1]
            z_pos = constVariable.CartesianPos[2]
            roll_pos = constVariable.CartesianPos[3]
            pitch_pos = constVariable.CartesianPos[4]
            yaw_pos = constVariable.CartesianPos[5] - speed * 10
            pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
            self.connection.moveCartasianPos(speed, pos)
        else:
            s_pos = constVariable.PulsePos[0]
            l_pos = constVariable.PulsePos[1]
            u_pos = constVariable.PulsePos[2]
            r_pos = constVariable.PulsePos[3]
            b_pos = constVariable.PulsePos[4]
            t_pos = constVariable.PulsePos[5] - speed * 10
            pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
            self.connection.movePulsePos(speed, pos)

    def incYaw(self):
        pos: list = []
        speed: int32 = self.uic.Speed_Slider.value() * 50
        if self.uic.Cartesian_coor.isChecked() == True:
            x_pos = constVariable.CartesianPos[0]
            y_pos = constVariable.CartesianPos[1]
            z_pos = constVariable.CartesianPos[2]
            roll_pos = constVariable.CartesianPos[3]
            pitch_pos = constVariable.CartesianPos[4]
            yaw_pos = constVariable.CartesianPos[5] + speed * 10
            pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
            self.connection.moveCartasianPos(speed, pos)
        else:
            s_pos = constVariable.PulsePos[0]
            l_pos = constVariable.PulsePos[1]
            u_pos = constVariable.PulsePos[2]
            r_pos = constVariable.PulsePos[3]
            b_pos = constVariable.PulsePos[4]
            t_pos = constVariable.PulsePos[5] + speed * 10
            pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
            self.connection.movePulsePos(speed, pos)
            
    def coorRadBtnCallback(self, select: bool):
        if select == True:
            self.uic.DecX_butt.setText("X-")
            self.uic.IncX_butt.setText("X+")
            self.uic.DecY_butt.setText("Y-")
            self.uic.IncY_butt.setText("Y+")
            self.uic.DecZ_butt.setText("Z-")
            self.uic.IncZ_butt.setText("Z+")
            self.uic.DecR_butt.setText("Roll-")
            self.uic.IncR_butt.setText("Roll+")
            self.uic.DecP_butt.setText("Pitch-")
            self.uic.IncP_butt.setText("Pitch+")
            self.uic.DecYaw_butt.setText("Yaw-")
            self.uic.IncYaw_butt.setText("Yaw+")

            self.uic.label_82.setText("X")
            self.uic.label_83.setText("Y")
            self.uic.label_84.setText("Z")
            self.uic.label_85.setText("Roll")
            self.uic.label_86.setText("Pitch")
            self.uic.label_87.setText("Yaw")

            self.uic.label_88.setText("mm")
            self.uic.label_89.setText("mm")
            self.uic.label_90.setText("mm")
        else:
            self.uic.DecX_butt.setText("S-")
            self.uic.IncX_butt.setText("S+")
            self.uic.DecY_butt.setText("L-")
            self.uic.IncY_butt.setText("L+")
            self.uic.DecZ_butt.setText("U-")
            self.uic.IncZ_butt.setText("U+")
            self.uic.DecR_butt.setText("R-")
            self.uic.IncR_butt.setText("R+")
            self.uic.DecP_butt.setText("B-")
            self.uic.IncP_butt.setText("B+")
            self.uic.DecYaw_butt.setText("T-")
            self.uic.IncYaw_butt.setText("T+")

            self.uic.label_82.setText("S")
            self.uic.label_83.setText("L")
            self.uic.label_84.setText("U")
            self.uic.label_85.setText("R")
            self.uic.label_86.setText("B")
            self.uic.label_87.setText("T")
            
            self.uic.label_88.setText("deg")
            self.uic.label_89.setText("deg")
            self.uic.label_90.setText("deg")
       
    @pyqtSlot(np.ndarray)
    def update_image(self, cv_img):
        """Updates the image_label with a new opencv image"""
        qt_img = self.convert_cv_qt(cv_img)
        self.uic.Frame.setPixmap(qt_img)

    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format.Format_RGB888)
        p = convert_to_Qt_format.scaled(640, 480, Qt.AspectRatioMode.KeepAspectRatio)
        return QPixmap.fromImage(p)
    
    """Display object name
    """
    @pyqtSlot(str) 
    def show_info(self, name):
        self.uic.text_name.setText(name)
        if name == "Kokomi":
            print("write byte 1")
            device.writeByte(21,1)
            
        if name == "Hao Hao":
            print("write byte 2")
            device.writeByte(21,2)
            
        if name == "Cung dinh":
            print("write byte 3")
            device.writeByte(21,3)
            
        if name == "Omachi":
            print("write byte 4")
            device.writeByte(21,4)
            
        if name == "Miliket":
            print("write byte 5")
            device.writeByte(21,5)
            
    """Display number of object
    """
    @pyqtSlot(str, str, str, str, str)
    def show_number(self, kokomi, cungdinh, haohao, omachi, miliket):
        self.uic.num_kokomi.setText(kokomi)
        self.uic.num_cungdinh.setText(cungdinh)
        self.uic.num_haohao.setText(haohao)
        self.uic.num_omachi.setText(omachi)
        self.uic.num_miliket.setText(miliket)

    """Display current position
    """  
    @pyqtSlot(str, str, str, str, str, str, str, str, str, str, str, str)
    def show_position(self, X, Y, Z, Roll, Pitch, Yaw, S, L, U, R, B, T):
        # print(X, Y, Z, Roll, Pitch, Yaw)
        self.uic.txt_X.setText(X)
        self.uic.txt_Y.setText(Y)
        self.uic.txt_Z.setText(Z)
        self.uic.txt_Roll.setText(Roll)
        self.uic.txt_Pitch.setText(Pitch)
        self.uic.txt_Yaw.setText(Yaw)
        
        self.uic.X_pos_text.setText(X)
        self.uic.Y_pos_text.setText(Y)
        self.uic.Z_pos_text.setText(Z)
        self.uic.Roll_pos_text.setText(Roll)
        self.uic.Pitch_pos_text.setText(Pitch)
        self.uic.Yaw_pos_text.setText(Yaw)
        
        self.uic.S_pos_text.setText(S)
        self.uic.L_pos_text.setText(L)
        self.uic.U_pos_text.setText(U)
        self.uic.R_pos_text.setText(R)
        self.uic.B_pos_text.setText(B)
        self.uic.T_pos_text.setText(T)
        
    @pyqtSlot(int)
    def show_speed(self, speed):
        self.uic.text_speed.setText(str(speed))
        