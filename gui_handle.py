# Use this file to control GUI
from PyQt5 import QtGui, QtSerialPort, QtCore
from PyQt5.QtSerialPort import QSerialPortInfo
from PyQt5.QtWidgets import QMainWindow, QApplication, QAction, QPlainTextEdit
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import  pyqtSlot, Qt, pyqtSignal, QTimer
import cv2, serial, sys
import numpy as np
import serial.tools.list_ports
from gui import Ui_MainWindow
from module import VideoThread
from robot_controller import Robot, GetPosition, UART
from motomini import Motomini

device = Motomini()
class MainWindow (QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Qt live label demo")
        self.uic = Ui_MainWindow()
        self.uic.setupUi(self)
        self.init_variables()
        self.init_button()
        
    def init_variables(self):
        self.thread = {}
        # Thread Status
        self.status_thread_1 = False
        self.status_thread_2 = False
        self.status_thread_3 = False
        
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
        
        self.uic.btn_load_job.clicked.connect(self.LOAD_JOB)
        self.uic.btn_start_job.clicked.connect(self.START_JOB)
        self.uic.btn_stop_job.clicked.connect(self.STOP_JOB)
        
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
            # self.ser = serial.Serial(self.com, self.baudrate, timeout = 2.5)
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
            # print(ip, port)
            # self.connection.connectMotomini(ip = ip, port = port)
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
        job = self.uic.text_JOB.text()
        line = 0
        self.uic.btn_load_job.setStyleSheet("QPushButton {color: green;}")
        # Start current Job at line 0
        self.connection.selectJob(job_name= job, line_no= line)
        self.job_status = True
                
    @checkConnect
    @checkServo
    def START_JOB(self):
        if self.job_status == True:
            self.connection.startJob()
            self.job_status = False
        else:
            print("Can not load job")
            
    @checkConnect
    @checkServo
    def STOP_JOB(self):
        self.uic.btn_load_job.setEnabled(True)
        self.uic.btn_stop_job.setEnabled(False)
        
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
        
    """Display number of object
    """
    @pyqtSlot(str,str,str,str,str)
    def show_number(self, gaudo, cungdinh, haohao, omachi102, omachispa):
        self.uic.num_gaudo.setText(gaudo)
        self.uic.num_cungdinh.setText(cungdinh)
        self.uic.num_haohao.setText(haohao)
        self.uic.num_omachi102.setText(omachi102)
        self.uic.num_omachispa.setText(omachispa)

    """Display current position
    """  
    @pyqtSlot(str,str,str,str,str,str)
    def show_position(self, X, Y, Z, Roll, Pitch, Yaw):
        print(X, Y, Z, Roll, Pitch, Yaw)
        self.uic.txt_X.setText(X)
        self.uic.txt_Y.setText(Y)
        self.uic.txt_Z.setText(Z)
        self.uic.txt_Roll.setText(Roll)
        self.uic.txt_Pitch.setText(Pitch)
        self.uic.txt_Yaw.setText(Yaw)
        
    @pyqtSlot(float)
    def show_speed(self, speed):
        print(speed)
        convoyer_speed = round(speed, 2)
        self.uic.text_speed.setText(convoyer_speed)
        