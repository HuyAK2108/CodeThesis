import cv2, serial
import numpy as np
import serial.tools.list_ports
from PyQt5 import QtGui
from PyQt5.QtWidgets import QMainWindow, QSlider, QAbstractItemView, QHeaderView, QTableWidgetItem
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import  pyqtSlot, Qt, QTimer
from gui import Ui_MainWindow
from module import VideoThread
from robot_controller import Robot, UART, Auto_system
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
        self.init_point_table()
        self.uic.btn_auto_system.setStyleSheet("QPushButton {color: green;}")
        
    def init_timer(self):
        self.timer_move_pos = QTimer()
        self.timer_move_pos.setInterval(100)
        self.timer_move_pos.timeout.connect(self.moveRobot)
        
    def init_variables(self):        
        # Init Thread
        self.thread = {}
        # Thread Status
        self.status_thread_1 = False
        self.status_thread_2 = False
        self.status_thread_3 = False
        self.status_thread_4 = False
        
        # Init Button Status
        self.uic.btn_servoON.setEnabled(False)
        self.uic.btn_home_pos.setEnabled(False)
        self.uic.btn_start_job.setEnabled(False)
        self.uic.btn_stop_job.setEnabled(False)
        self.uic.btn_offCAM.setEnabled(False)
        self.uic.btn_closeuart.setEnabled(False)
        # Init Light status
        self.uic.lb_on_cam.hide()
        self.uic.lb_on_robot.hide()
        self.uic.lb_wait_robot.hide()
        self.uic.lb_on_conveyor.hide()
        self.uic.lb_on_serial.hide()
        self.uic.lb_auto_status.hide()
        # Connection Status
        self.job_status             = False
        self.com_connect_status     = False
        self.robot_status           = False
        self.conveyor_status        = False
        self.auto_status            = False
        # Call Motomini class
        self.connection = device
        # Table
        self.STT_count_1 = 1
        self.STT_count_2 = 1
        self.point_count_1 = 0
        self.point_count_2 = 0
        self.row_init_1 = 7
        self.row_init_2 = 7
        # Get available serial ports
        self.comlist = serial.tools.list_ports.comports()
        self.connected = []
        for element in self.comlist:
            self.connected.append(element.device)
        self.uic.COM.addItems(self.connected)        
    
    def init_point_table(self) -> None:
        self.uic.Point_teach_1.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.uic.Point_teach_1.setSelectionMode(QAbstractItemView.NoSelection)
        self.uic.Point_teach_1.setColumnWidth(0, 75)
        self.uic.Point_teach_1.setColumnWidth(1, 60)
        self.uic.Point_teach_1.setColumnWidth(2, 60)
        self.uic.Point_teach_1.setColumnWidth(3, 60)
        self.uic.Point_teach_1.setColumnWidth(4, 60)
        self.uic.Point_teach_1.setColumnWidth(5, 60)
        self.uic.Point_teach_1.setColumnWidth(6, 60)
        self.uic.Point_teach_1.horizontalHeader().setSectionResizeMode(QHeaderView.Fixed)
        # Add row
        self.uic.Point_teach_1.verticalHeader().setSectionsClickable(False)
        self.uic.Point_teach_1.verticalHeader().setDefaultSectionSize(15)
        self.uic.Point_teach_1.verticalHeader().setVisible(False)
        for x in range(self.row_init_1):
            self.uic.Point_teach_1.insertRow(x)
            
        self.uic.Point_teach_2.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.uic.Point_teach_2.setSelectionMode(QAbstractItemView.NoSelection)
        self.uic.Point_teach_2.setColumnWidth(0, 75)
        self.uic.Point_teach_2.setColumnWidth(1, 60)
        self.uic.Point_teach_2.setColumnWidth(2, 60)
        self.uic.Point_teach_2.setColumnWidth(3, 60)
        self.uic.Point_teach_2.setColumnWidth(4, 60)
        self.uic.Point_teach_2.setColumnWidth(5, 60)
        self.uic.Point_teach_2.setColumnWidth(6, 60)
        self.uic.Point_teach_2.horizontalHeader().setSectionResizeMode(QHeaderView.Fixed)
        # Add row
        self.uic.Point_teach_2.verticalHeader().setSectionsClickable(False)
        self.uic.Point_teach_2.verticalHeader().setDefaultSectionSize(15)
        self.uic.Point_teach_2.verticalHeader().setVisible(False)
        for y in range(self.row_init_2):
            self.uic.Point_teach_2.insertRow(y)
            
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
        # Button reload P101 - 105
        self.uic.btn_auto_system.clicked.connect(self.AUTO_SYSTEM)
        # Button Teaching point
        self.uic.btn_teach.clicked.connect(self.TEACH_POINT)
        self.uic.DelRow_butt.clicked.connect(self.delete_allrow_table)
        # Button ON - OFF conveyor
        self.uic.btn_conveyor.clicked.connect(self.CONVEYOR)
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
        self.uic.lb_on_cam.show()
        self.uic.lb_off_cam.hide()
        self.uic.lb_camera_status.setText("ON")
        self.status_thread_1 = True
        # create the video capture thread
        self.thread[1] = VideoThread(index=1)
        # connect its signal to the show_info slot to display object name
        self.thread[1].signal.connect(self.show_info)
        self.thread[1].number.connect(self.show_number)
        # connect its signal to the update_image slot to display webcam
        self.thread[1].change_pixmap_signal.connect(self.update_image)
        self.thread[1].position.connect(self.get_object_position)
        # start the thread
        self.thread[1].start()
                
    def STOP_CAPTURE_VIDEO(self):
        """Stop capture video and turn off camera
        """
        self.uic.btn_onCAM.setEnabled(True)
        self.uic.btn_offCAM.setEnabled(False)
        self.uic.btn_onCAM.setStyleSheet("QPushButton {color: black;}")
        self.uic.btn_offCAM.setStyleSheet("QPushButton {color: red;}")
        self.uic.lb_camera_status.setText("OFF")
        self.uic.lb_on_cam.hide()
        self.uic.lb_off_cam.show()
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
        if self.status_thread_4 == True:
            self.thread[4].stop()
        event.accept()
        
    def CONNECT_SERIAL(self):
        """Open UART to send/receive data
        """
        self.com = self.uic.COM.currentText()
        self.baudrate = int(self.uic.Baudrate.currentText())
        if self.com_connect_status == False:
            self.status_thread_3 = True
            self.com_connect_status = True
            self.uic.btn_closeuart.setEnabled(True)
            self.uic.btn_setuart.setEnabled(False)
            self.uic.lb_serial_status.setText("ON")
            self.uic.lb_on_serial.show()
            self.uic.lb_off_serial.hide()
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
        self.uic.lb_serial_status.setText("OFF")
        self.uic.lb_off_serial.show()
        self.uic.lb_on_serial.hide()
        self.com_connect_status = False
        self.thread[3].stop_timer()
        self.thread[3].stop()
    
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
                print("Warning!!! Servo is off")
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
                self.uic.lb_run_status.setText("ON")
                self.uic.lb_on_robot.show()
                self.uic.lb_wait_robot.hide()
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
                self.uic.lb_run_status.setText("OFF")
                self.uic.lb_wait_robot.show()
                self.uic.lb_on_robot.hide()
                self.thread[2].stop_receive_pos()
            else:
                print("Can not off servo")
        
    def CONNECT_ROBOT(self):        
        if self.connection.checkConnectStatus() == False:
            # ip = self.uic.text_IP.text()
            # port = int(self.uic.text_Port.text())
            # self.connection.connectMotomini(ip = ip, port = port)
            self.connection.connectMotomini(ip = "192.168.1.12", port = 10040)
            self.uic.btn_servoON.setEnabled(True)
            self.uic.btn_home_pos.setEnabled(True)
            self.uic.lb_wait_robot.show()
            self.uic.lb_off_robot.hide()
            self.robot_status = True
            self.uic.btn_setconnnect.setText("DISCONNECT")
            self.uic.btn_setconnnect.setStyleSheet("QPushButton {color: red;}")
            print("Connected to Robot")
        else:
            self.connection.disconnectMotomini()
            self.uic.lb_off_robot.show()
            self.uic.lb_wait_robot.hide()
            self.uic.lb_on_robot.hide()
            self.robot_status = False
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
        device.writeByte(21, 6)
       
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
        flag.name = name
        if flag.flag_setName != [0,0,0,0,0]:   
            self.uic.text_name.setText(name)
            
    """Display number of object
    """
    @pyqtSlot(str, str, str, str, str)
    def show_number(self, kokomi, cungdinh, haohao, omachi, miliket):
        self.uic.num_kokomi.setText(kokomi)
        self.uic.num_cungdinh.setText(cungdinh)
        self.uic.num_haohao.setText(haohao)
        self.uic.num_omachi.setText(omachi)
        self.uic.num_miliket.setText(miliket)
        CountObject.cung_dinh   = cungdinh
        CountObject.hao_hao     = haohao
        CountObject.omachi      = omachi
        CountObject.miliket     = miliket
        CountObject.kokomi      = kokomi

    """Display current position
    """  
    @pyqtSlot(str, str, str, str, str, str, str, str, str, str, str, str, int)
    def show_position(self, X, Y, Z, Roll, Pitch, Yaw, S, L, U, R, B, T, B022):
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
        
        Byte.B022 = B022
        
    @pyqtSlot(int)
    def show_speed(self, speed):
        self.uic.text_speed.setText(str(speed))
        conveyor.speed = speed
           
    @pyqtSlot(int, int, int, int, int)
    def get_object_position(self, flag_kokomi, flag_cungdinh, flag_haohao, flag_omachi, flag_miliket):
        flag.flag_kokomi   = flag_kokomi
        flag.flag_cungdinh = flag_cungdinh
        flag.flag_haohao   = flag_haohao
        flag.flag_omachi   = flag_omachi
        flag.flag_miliket  = flag_miliket
        flag.flag_setName = [flag_cungdinh, flag_haohao, flag_kokomi, flag_miliket, flag_omachi]
        
        # if self.robot_status == True:
        #     x_pos       =  250  * 1000
        #     y_pos       = -150  * 1000
        #     z_pos       = -120  * 1000
        #     roll_pos    = -180  * 10000
        #     pitch_pos   =  0    * 10000
        #     yaw_pos     =  0    * 1000

        #     pos_pick = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
            
        #     if flag.name == "Cung dinh":
        #         if flag.flag_cungdinh == 1:
        #             print("write byte 1")
        #             pos = init_pos.P101
        #             # pos[2] += 6000 * int(CountObject.cung_dinh)
        #             device.writeVariablePos(101, pos)
        #             device.writeVariablePos(121, pos_pick)
        #             device.writeByte(21,1)
                    
        #     elif flag.name == "Hao Hao":
        #         if flag.flag_haohao == 1:
        #             print("write byte 2")
        #             pos = init_pos.P102
        #             # pos[2] += 6000 * int(CountObject.hao_hao)
        #             device.writeVariablePos(102, pos)
        #             device.writeVariablePos(121, pos_pick)
        #             device.writeByte(21,2)

        #     elif flag.name == "Kokomi":
        #         if flag.flag_kokomi == 1:
        #             print("write byte 3")
        #             pos = init_pos.P103
        #             # pos[2] += 6000 * int(CountObject.kokomi)
        #             device.writeVariablePos(103, pos)
        #             device.writeVariablePos(121, pos_pick)
        #             device.writeByte(21,3)

        #     elif flag.name == "Miliket":
        #         if flag.flag_miliket == 1:
        #             print("write byte 4")
        #             pos = init_pos.P104
        #             # pos[2] += 6000 * int(CountObject.miliket)
        #             device.writeVariablePos(104, pos)
        #             device.writeVariablePos(121, pos_pick)
        #             device.writeByte(21,4)

        #     elif flag.name == "Omachi":
        #         if flag.flag_omachi == 1:
        #             print("write byte 5")
        #             pos = init_pos.P105
        #             # pos[2] += 6000 * int(CountObject.omachi)
        #             device.writeVariablePos(105, pos)
        #             device.writeVariablePos(121, pos_pick)
        #             device.writeByte(21,5)
        #     else:
        #         print("No object detected !")
                    
    def AUTO_SYSTEM(self):
        if self.status_thread_4 == False:
            self.thread[4] = Auto_system(index = 4, robot_status= self.robot_status)
            self.thread[4].start_program()
            self.thread[4].timer_count.connect(self.counter)
            self.status_thread_4 = True

            self.uic.btn_auto_system.setText("STOP")
            self.uic.btn_auto_system.setStyleSheet("QPushButton {color: red;}")
            self.uic.lb_on_auto.show()
            self.uic.lb_off_auto.hide()
            self.uic.lb_auto_status.setText("ON")
        else:
            self.thread[4].stop_program()
            self.status_thread_4 = False

            self.uic.btn_auto_system.setText("START")
            self.uic.btn_auto_system.setStyleSheet("QPushButton {color: green;}")
            self.uic.lb_off_auto.show()
            self.uic.lb_on_auto.hide()
            self.uic.lb_auto_status.setText("OFF")
    
    @pyqtSlot(float)
    def counter(self, count):
        counter = round(count,2)
        delta_y = conveyor.speed * int(counter)
        delta_y = 38 * int(counter) # Cover missing STM32 and UART
        print("Time:",counter)
        init_pos.P121[1] += delta_y   
        print("Location Y to pick:", init_pos.P121[1])
        pos_pick = init_pos.P121
        
        if flag.name == "Cung dinh":
            if flag.flag_cungdinh == 1:
                print("write byte 1")
                pos = init_pos.P101
                # pos[2] += 6000 * int(CountObject.cung_dinh)
                device.writeVariablePos(101, pos)
                device.writeVariablePos(121, pos_pick)
                device.writeByte(21,1)
                
        elif flag.name == "Hao Hao":
            if flag.flag_haohao == 1:
                print("write byte 2")
                pos = init_pos.P102
                # pos[2] += 6000 * int(CountObject.hao_hao)
                device.writeVariablePos(102, pos)
                device.writeVariablePos(121, pos_pick)
                device.writeByte(21,2)
                
        elif flag.name == "Kokomi":
            if flag.flag_kokomi == 1:
                print("write byte 3")
                pos = init_pos.P103
                # pos[2] += 6000 * int(CountObject.kokomi)
                device.writeVariablePos(103, pos)
                device.writeVariablePos(121, pos_pick)
                device.writeByte(21,3)
                
        elif flag.name == "Miliket":
            if flag.flag_miliket == 1:
                print("write byte 4")
                pos = init_pos.P104
                # pos[2] += 6000 * int(CountObject.miliket)
                device.writeVariablePos(104, pos)
                device.writeVariablePos(121, pos_pick)
                device.writeByte(21,4)
                
        elif flag.name == "Omachi":
            if flag.flag_omachi == 1:
                print("write byte 5")
                pos = init_pos.P105
                # pos[2] += 6000 * int(CountObject.omachi)
                device.writeVariablePos(105, pos)
                device.writeVariablePos(121, pos_pick)
                device.writeByte(21,5)
        
    def RELOAD_POSITION(self):
        # Load initial value of P101 - 105 and P110
        device.writeVariablePos(101, init_pos.P101)
        device.writeVariablePos(102, init_pos.P102)
        device.writeVariablePos(103, init_pos.P103)
        device.writeVariablePos(104, init_pos.P104)
        device.writeVariablePos(105, init_pos.P105)
        device.writeVariablePos(110, init_pos.P110)
    
    def add_row_table_1(self) -> None:
        row_count_1 = self.uic.Point_teach_1.rowCount()
        self.uic.Point_teach_1.insertRow(row_count_1)
        
    def add_row_table_2(self) -> None:
        row_count_2 = self.uic.Point_teach_2.rowCount()
        self.uic.Point_teach_2.insertRow(row_count_2)

    def delete_allrow_table(self) -> None:
        self.uic.Point_teach_1.setRowCount(0)
        self.uic.Point_teach_2.setRowCount(0)
        self.init_point_table()
        self.STT_count_1 = 1
        self.STT_count_2 = 1
        self.point_count_1 = 0
        self.point_count_2 = 0
     
    def TEACH_POINT(self):
        if self.point_count_1 >= self.uic.Point_teach_1.rowCount():
            self.add_row_table_1()
        if self.point_count_2 >= self.uic.Point_teach_2.rowCount():
            self.add_row_table_2()
        
        if self.uic.Cartesian_coor.isChecked() == True:  
            item = QTableWidgetItem()
            item.setText(self.uic.txt_pos.text())
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.uic.Point_teach_2.setItem(self.point_count_2, tableColumn_2.STT.value, item)
            
            item = QTableWidgetItem()
            item.setText(str(self.uic.S_move_text.text()))
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.uic.Point_teach_2.setItem(self.point_count_2, tableColumn_2.X_col.value, item)
            item = QTableWidgetItem()
            
            item.setText(str(self.uic.L_move_text.text()))
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.uic.Point_teach_2.setItem(self.point_count_2, tableColumn_2.Y_col.value, item)

            item = QTableWidgetItem()
            item.setText(str(self.uic.U_move_text.text()))
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.uic.Point_teach_2.setItem(self.point_count_2, tableColumn_2.Z_col.value, item)

            item = QTableWidgetItem()
            item.setText(str(self.uic.R_move_text.text()))
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.uic.Point_teach_2.setItem(self.point_count_2, tableColumn_2.Roll_col.value, item)

            item = QTableWidgetItem()
            item.setText(str(self.uic.B_move_text.text()))
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.uic.Point_teach_2.setItem(self.point_count_2, tableColumn_2.Pitch_col.value, item)

            item = QTableWidgetItem()
            item.setText(str(self.uic.T_move_text.text()))
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.uic.Point_teach_2.setItem(self.point_count_2, tableColumn_2.Yaw_col.value, item)
            
            self.point_count_2 += 1
            self.STT_count_2 += 1
            
            x_pos       = int( float (self.uic.S_move_text.text()) * 1000)
            y_pos       = int( float (self.uic.L_move_text.text()) * 1000)
            z_pos       = int( float (self.uic.U_move_text.text()) * 1000)
            roll_pos    = int( float (self.uic.R_move_text.text()) * 10000)
            pitch_pos   = int( float (self.uic.B_move_text.text()) * 10000)
            yaw_pos     = int( float (self.uic.T_move_text.text()) * 10000)
            pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
            position_reg = int(self.uic.txt_pos.text())
            print('P' + str(position_reg) +': '+ str(pos))
            device.writeVariablePos(position_reg, pos)
        
        else:         
            item = QTableWidgetItem()
            item.setText(self.uic.txt_pos.text())
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.uic.Point_teach_1.setItem(self.point_count_1, tableColumn.STT.value, item)
            
            item = QTableWidgetItem()
            item.setText(str(self.uic.S_move_text.text()))
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.uic.Point_teach_1.setItem(self.point_count_1, tableColumn.S_col.value, item)

            item = QTableWidgetItem()
            item.setText(str(self.uic.L_move_text.text()))
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.uic.Point_teach_1.setItem(self.point_count_1, tableColumn.L_col.value, item)

            item = QTableWidgetItem()
            item.setText(str(self.uic.U_move_text.text()))
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.uic.Point_teach_1.setItem(self.point_count_1, tableColumn.U_col.value, item)

            item = QTableWidgetItem()
            item.setText(str(self.uic.R_move_text.text()))
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.uic.Point_teach_1.setItem(self.point_count_1, tableColumn.R_col.value, item)

            item = QTableWidgetItem()
            item.setText(str(self.uic.B_move_text.text()))
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.uic.Point_teach_1.setItem(self.point_count_1, tableColumn.B_col.value, item)

            item = QTableWidgetItem()
            item.setText(str(self.uic.T_move_text.text()))
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.uic.Point_teach_1.setItem(self.point_count_1, tableColumn.T_col.value, item)
  
            self.point_count_1 += 1
            self.STT_count_1 += 1
     
            s_pos = int( float (self.uic.S_move_text.text()) * constVariable.pulse_per_degree_S)
            l_pos = int( float (self.uic.L_move_text.text()) * constVariable.pulse_per_degree_L)
            u_pos = int( float (self.uic.U_move_text.text()) * constVariable.pulse_per_degree_U)
            r_pos = int( float (self.uic.R_move_text.text()) * constVariable.pulse_per_degree_RBT)
            b_pos = int( float (self.uic.B_move_text.text()) * constVariable.pulse_per_degree_RBT)
            t_pos = int( float (self.uic.T_move_text.text()) * constVariable.pulse_per_degree_RBT)
            pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
            position_reg = int(self.uic.txt_pos.text())
            print('P' + str(position_reg) +': '+ str(pos))
            device.writeVariablePos(position_reg, pos)
            
    # MANUAL
    def CONVEYOR(self):
        if self.conveyor_status == False:
            device.writeByte(9, 1)
            self.uic.btn_conveyor.setText("OFF")
            self.uic.btn_conveyor.setStyleSheet("QPushButton {color: red;}")
            self.uic.lb_conveyor_status.setText("ON")
            self.uic.lb_on_conveyor.show()
            self.uic.lb_off_conveyor.hide()
            self.conveyor_status = True
        else:
            device.writeByte(9, 0)
            self.uic.btn_conveyor.setText("ON")
            self.uic.btn_conveyor.setStyleSheet("QPushButton {color: green;}")
            self.uic.lb_conveyor_status.setText("OFF")
            self.uic.lb_off_conveyor.show()
            self.uic.lb_on_conveyor.hide()
            self.conveyor_status = False
            
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
            x_pos       = int( float (self.uic.S_move_text.text()) * 1000)
            y_pos       = int( float (self.uic.L_move_text.text()) * 1000)
            z_pos       = int( float (self.uic.U_move_text.text()) * 1000)
            roll_pos    = int( float (self.uic.R_move_text.text()) * 10000)
            pitch_pos   = int( float (self.uic.B_move_text.text()) * 10000)
            yaw_pos     = int( float (self.uic.T_move_text.text()) * 10000)
            
            pos = [x_pos, y_pos, z_pos, roll_pos, pitch_pos, yaw_pos]
            
            self.connection.moveCartasianPos(speed, pos)
            
        else:
            s_pos = int( float (self.uic.S_move_text.text()) * constVariable.pulse_per_degree_S)
            l_pos = int( float (self.uic.L_move_text.text()) * constVariable.pulse_per_degree_L)
            u_pos = int( float (self.uic.U_move_text.text()) * constVariable.pulse_per_degree_U)
            r_pos = int( float (self.uic.R_move_text.text()) * constVariable.pulse_per_degree_RBT)
            b_pos = int( float (self.uic.B_move_text.text()) * constVariable.pulse_per_degree_RBT)
            t_pos = int( float (self.uic.T_move_text.text()) * constVariable.pulse_per_degree_RBT)
            
            pos = [s_pos, l_pos, u_pos, r_pos, b_pos, t_pos]
            
            self.connection.movePulsePos(speed, pos)
    
    # Run by pressing 
    def decX(self):
        pos: list = []
        speed: int32 = self.uic.Speed_Slider.value() * 50
        if self.uic.Cartesian_coor.isChecked() == True:
            x_pos       = constVariable.CartesianPos[0] - speed * 10
            y_pos       = constVariable.CartesianPos[1]
            z_pos       = constVariable.CartesianPos[2]
            roll_pos    = constVariable.CartesianPos[3]
            pitch_pos   = constVariable.CartesianPos[4]
            yaw_pos     = constVariable.CartesianPos[5]
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