import cv2, serial, time
import numpy as np
import serial.tools.list_ports
from PyQt5 import QtGui
from PyQt5.QtWidgets import QMainWindow, QSlider, QAbstractItemView, QHeaderView, QTableWidgetItem, QMessageBox
from PyQt5.QtGui import QPixmap, QIcon
from PyQt5.QtCore import  pyqtSlot, Qt, QTimer, QSize, QThread
from gui import Ui_MainWindow
from module import VideoThread
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

    def init_timer(self):
        self.timer_move_pos = QTimer()
        self.timer_move_pos.setInterval(300)
        self.timer_move_pos.timeout.connect(self.moveRobot)
        
        self.timer_update_gui = QTimer()
        self.timer_update_gui.setInterval(300)
        self.timer_update_gui.timeout.connect(self.updateGUI)
        
        self.auto_system = QTimer()
        self.auto_system.setInterval(50)
        self.auto_system.timeout.connect(self.auto_program)
        
        self.TIMER_UART = QTimer()
        self.TIMER_UART.setInterval(500)
        self.TIMER_UART.timeout.connect(self.get_data)
        
    def init_variables(self):        
        # Thread Status
        self.status_thread_1 = False
        # Init Button Status
        self.uic.btn_on_trigger.setIcon(QIcon("pictures/off.jpg"))
        self.uic.btn_on_conveyor.setIcon(QIcon("pictures/off.jpg"))
        self.uic.btn_on_bgs.setIcon(QIcon("pictures/off.jpg"))
        self.uic.btn_on_bw.setIcon(QIcon("pictures/off.jpg"))
        self.uic.btn_on_bgs.setIconSize(QSize(80,15))
        self.uic.btn_on_trigger.setIconSize(QSize(80,15))
        self.uic.btn_on_conveyor.setIconSize(QSize(80,15))
        self.uic.btn_on_bw.setIconSize(QSize(80,15))
        self.uic.btn_on_conveyor.setStyleSheet("border : 1px solid white")
        self.uic.btn_on_trigger.setStyleSheet("border : 1px solid white")
        self.uic.btn_on_bgs.setStyleSheet("border : 1px solid white")
        self.uic.btn_on_bw.setStyleSheet("border : 1px solid white")
        # Init Light status
        self.uic.lb_on_cam.hide()
        self.uic.lb_on_robot.hide()
        self.uic.lb_wait_robot.hide()
        self.uic.lb_on_conveyor.hide()
        self.uic.lb_on_serial.hide()
        self.uic.lb_on_auto.hide()
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
        # Button Get Position
        self.uic.btn_get_pos.clicked.connect(self.GET_VARIABLE_POS)
        # Button Reset Count Object
        self.uic.btn_reset_count.clicked.connect(self.RESET_COUNT_OBJECT)
        # Button ON - OFF camera
        self.uic.btn_onCAM.clicked.connect(self.START_CAPTURE_VIDEO)
        self.uic.btn_offCAM.clicked.connect(self.STOP_CAPTURE_VIDEO)
        self.uic.btn_on_trigger.clicked.connect(self.ON_OFF_TRIGGER_LINE)
        self.uic.btn_on_conveyor.clicked.connect(self.ON_OFF_CONVEYOR_LINE)
        self.uic.btn_on_bgs.clicked.connect(self.ON_OFF_BGS)
        self.uic.btn_on_bw.clicked.connect(self.ON_OFF_BW)
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
            
    def checkConnect(func):
        def check(self):
            if self.connection.checkConnectStatus() == False:
                self.error_msg("No connection with robot\n"+"Check the connection !")
                return
            return func(self)
        return check

    def checkServo(func):
        def check(self):
            if self.connection.checkServoStatus() == False:
                self.error_msg("Servo is off\n"+"Check the connection !")
                return
            return func(self)
        return check
    
    def GET_VARIABLE_POS(self):
        num_pos = int(self.uic.txt_get_pos.text())
        device.getVariablePos(index= num_pos)
        device.convertGetPos()
        X     = (str (round (float(constVariable.VariablePos[0]) / 1000 , 3 ) ) )
        Y     = (str (round (float(constVariable.VariablePos[1]) / 1000 , 3 ) ) )
        Z     = (str (round (float(constVariable.VariablePos[2]) / 1000 , 3 ) ) )
        Roll  = (str (round (float(constVariable.VariablePos[3]) / 10000, 4 ) ) )
        Pitch = (str (round (float(constVariable.VariablePos[4]) / 10000, 4 ) ) )
        Yaw   = (str (round (float(constVariable.VariablePos[5]) / 10000, 4 ) ) )
        
        if self.point_count_2 >= self.uic.Point_teach_2.rowCount():
            self.add_row_table_2()
          
        item = QTableWidgetItem()
        item.setText(self.uic.txt_get_pos.text())
        item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.uic.Point_teach_2.setItem(self.point_count_2, tableColumn_2.STT.value, item)
        
        item = QTableWidgetItem()
        item.setText(X)
        item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.uic.Point_teach_2.setItem(self.point_count_2, tableColumn_2.X_col.value, item)
        item = QTableWidgetItem()
        
        item.setText(Y)
        item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.uic.Point_teach_2.setItem(self.point_count_2, tableColumn_2.Y_col.value, item)
        item = QTableWidgetItem()
        
        item.setText(Z)
        item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.uic.Point_teach_2.setItem(self.point_count_2, tableColumn_2.Z_col.value, item)
        item = QTableWidgetItem()
        
        item.setText(Roll)
        item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.uic.Point_teach_2.setItem(self.point_count_2, tableColumn_2.Roll_col.value, item)
        item = QTableWidgetItem()
        
        item.setText(Pitch)
        item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.uic.Point_teach_2.setItem(self.point_count_2, tableColumn_2.Pitch_col.value, item)
        item = QTableWidgetItem()
        
        item.setText(Yaw)
        item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.uic.Point_teach_2.setItem(self.point_count_2, tableColumn_2.Yaw_col.value, item)
        
        self.point_count_2 += 1
        self.STT_count_2   += 1     
        
    def RESET_COUNT_OBJECT(self):
        CountObject.bistro      = 0
        CountObject.cung_dinh   = 0
        CountObject.hao_hao     = 0
        CountObject.kokomi      = 0
        CountObject.omachi      = 0
        
    def ON_OFF_TRIGGER_LINE(self):
        if flag.trigger == False:
            flag.trigger = True
            self.uic.btn_on_trigger.setIcon(QIcon("pictures/on.jpg"))
        else:
            flag.trigger = False
            self.uic.btn_on_trigger.setIcon(QIcon("pictures/off.jpg"))   
    
    def ON_OFF_CONVEYOR_LINE(self):
        if flag.conveyor == False:
            flag.conveyor = True
            self.uic.btn_on_conveyor.setIcon(QIcon("pictures/on.jpg"))
        else:
            flag.conveyor = False
            self.uic.btn_on_conveyor.setIcon(QIcon("pictures/off.jpg"))
    
    def ON_OFF_BGS(self):
        if flag.bgs == False:
            flag.bgs = True
            self.uic.btn_on_bgs.setIcon(QIcon("pictures/on.jpg"))
        else:
            flag.bgs = False
            self.uic.btn_on_bgs.setIcon(QIcon("pictures/off.jpg"))
            
    def ON_OFF_BW(self):
        if flag.bw == False:
            flag.bw = True
            self.uic.btn_on_bw.setIcon(QIcon("pictures/on.jpg"))
        else:
            flag.bw = False
            self.uic.btn_on_bw.setIcon(QIcon("pictures/off.jpg"))
    
    def updateGUI(self):
        device.getCartasianPos()
        time.sleep(0.0005)
        device.getPulsePos()
        time.sleep(0.0005)
        device.convertPos()
        device.convertPulsePos()
          
        X     = (str (round (float(constVariable.CartesianPos[0]) / 1000 , 3 ) ) )
        Y     = (str (round (float(constVariable.CartesianPos[1]) / 1000 , 3 ) ) )
        Z     = (str (round (float(constVariable.CartesianPos[2]) / 1000 , 3 ) ) )
        Roll  = (str (round (float(constVariable.CartesianPos[3]) / 10000, 4) ) )
        Pitch = (str (round (float(constVariable.CartesianPos[4]) / 10000, 4) ) )
        Yaw   = (str (round (float(constVariable.CartesianPos[5]) / 10000, 4) ) )
        
        S     = (str (round (float(constVariable.PulsePos[0]) / constVariable.pulse_per_degree_S   ,3 ) ) )
        L     = (str (round (float(constVariable.PulsePos[1]) / constVariable.pulse_per_degree_L   ,3 ) ) )
        U     = (str (round (float(constVariable.PulsePos[2]) / constVariable.pulse_per_degree_U   ,3 ) ) )
        R     = (str (round (float(constVariable.PulsePos[3]) / constVariable.pulse_per_degree_RBT ,3 ) ) )
        B     = (str (round (float(constVariable.PulsePos[4]) / constVariable.pulse_per_degree_RBT ,3 ) ) )
        T     = (str (round (float(constVariable.PulsePos[5]) / constVariable.pulse_per_degree_RBT ,3 ) ) )
        
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
        if self.status_thread_1 == False:
            self.uic.btn_onCAM.setEnabled(False)
            self.uic.btn_offCAM.setEnabled(True)
            self.uic.btn_onCAM.setStyleSheet("QPushButton {color: green;}")
            self.uic.btn_offCAM.setStyleSheet("QPushButton {color: black;}")
            self.uic.lb_on_cam.show()
            self.uic.lb_off_cam.hide()
            self.uic.lb_camera_status.setText("ON")
            self.status_thread_1 = True
            # create the video capture thread
            self.thread = VideoThread(index=1)
            # connect its signal to the show_info slot to display object name
            self.thread.label_signal.connect(self.show_info)
            self.thread.flag_signal.connect(self.show_object_flag)
            # connect its signal to the update_image slot to display webcam
            self.thread.change_pixmap_signal.connect(self.update_image)
            # start the thread
            self.thread.start(QThread.Priority.HighestPriority)
                
    def STOP_CAPTURE_VIDEO(self):
        """Stop capture video and turn off camera
        """
        if self.status_thread_1 == True:
            self.status_thread_1 = False
            self.uic.btn_onCAM.setEnabled(True)
            self.uic.btn_offCAM.setEnabled(False)
            self.uic.btn_onCAM.setStyleSheet("QPushButton {color: black;}")
            self.uic.btn_offCAM.setStyleSheet("QPushButton {color: red;}")
            self.uic.lb_on_cam.hide()
            self.uic.lb_off_cam.show()
            self.uic.lb_camera_status.setText("OFF")
            self.thread.stop()
        else:
            self.error_msg("Camera is not open\n"+"Check the connection !")
            
    def closeEvent(self, event):
        """Close Window
        """
        if self.status_thread_1 == True:
            self.thread.stop()
            
        if self.com_connect_status == True:
            self.TIMER_UART.stop()
            
        if self.connection.checkServoStatus() == True:
            self.timer_update_gui.stop()
            self.auto_system.stop()  
        event.accept()
        
    def CONNECT_SERIAL(self):
        """Open UART to send/receive data
        """
        self.com = self.uic.COM.currentText()
        self.baudrate = int(self.uic.Baudrate.currentText())
        if self.com_connect_status == False:
            self.ser = serial.Serial(self.com, self.baudrate, timeout = 2.5)
            self.com_connect_status = True
            self.uic.btn_closeuart.setEnabled(True)
            self.uic.btn_setuart.setEnabled(False)
            self.uic.lb_serial_status.setText("ON")
            self.uic.lb_on_serial.show()
            self.uic.lb_off_serial.hide()
            self.TIMER_UART.start()
        
    def DISCONNECT_SERIAL(self):
        """Close UART
        """
        if self.com_connect_status == True:
            self.uic.btn_closeuart.setEnabled(False)
            self.uic.btn_setuart.setEnabled(True)
            self.uic.lb_serial_status.setText("OFF")
            self.uic.lb_off_serial.show()
            self.uic.lb_on_serial.hide()
            self.com_connect_status = False
            self.ser.close() 
            self.TIMER_UART.stop()
        else:
            self.error_msg("Serial is not connected\n"+"Check the connection !")
            
    def get_data(self):
        bytetoread = []
        if self.com_connect_status == True:
            bytetoread = self.ser.inWaiting()
            if bytetoread > 0:
                RXData = self.ser.readline(bytetoread)
                speed = RXData[0]
            else:
                speed = 0
            self.uic.text_speed.setText(str(speed))
    
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
                self.uic.lb_on_robot.show()
                self.uic.lb_wait_robot.hide()
                self.uic.lb_on_auto.show()
                self.uic.lb_off_auto.hide()
                self.uic.lb_run_status.setText("ON")
                self.uic.lb_auto_status.setText("ON")
                
                self.timer_update_gui.start()
                self.auto_system.start()
            else:
                self.error_msg("Can not on servo !\n"+"Check the connection !")
        else:
            if self.connection.offServo() == 0:
                self.uic.btn_servoON.setText("SERVO ON")
                self.uic.btn_servoON.setStyleSheet("QPushButton {color: green;}")
                self.uic.lb_wait_robot.show()
                self.uic.lb_on_robot.hide()
                self.uic.lb_off_auto.show()
                self.uic.lb_on_auto.hide()
                self.uic.lb_run_status.setText("OFF")
                self.uic.lb_auto_status.setText("OFF")
                
                self.timer_update_gui.stop()
                self.auto_system.stop()
            else:
                self.error_msg("Can not off servo !\n"+"Check the connection !")
        
    def CONNECT_ROBOT(self):        
        if self.connection.checkConnectStatus() == False:
            self.connection.connectMotomini(ip = "192.168.1.12", port = 10040)
            self.uic.btn_servoON.setEnabled(True)
            self.uic.btn_home_pos.setEnabled(True)
            self.uic.lb_wait_robot.show()
            self.uic.lb_off_robot.hide()
            self.robot_status = True
            self.uic.btn_setconnnect.setText("DISCONNECT")
            self.uic.btn_setconnnect.setStyleSheet("QPushButton {color: red;}")
        else:
            self.connection.disconnectMotomini()
            self.uic.lb_off_robot.show()
            self.uic.lb_wait_robot.hide()
            self.uic.lb_on_robot.hide()
            self.robot_status = False
            self.uic.btn_setconnnect.setText("CONNECT")
            self.uic.btn_setconnnect.setStyleSheet("QPushButton {color: green;}")
    
    @checkConnect
    @checkServo
    def LOAD_JOB(self):
        self.uic.btn_load_job.setEnabled(False)
        self.uic.btn_stop_job.setEnabled(True)
        self.uic.btn_start_job.setEnabled(True)
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
            self.error_msg("Can not load job\n"+"Check the connection !")
            
    @checkConnect
    @checkServo
    def STOP_JOB(self):
        self.job_status = False
        self.uic.btn_load_job.setEnabled(True)
        self.uic.btn_stop_job.setEnabled(False)
        self.uic.btn_start_job.setEnabled(False)
        device.writeByte(21, 6)
        device.writeByte(9 , 0)
       
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
        if flag.flag_setName != [0,0,0,0,0]:   
            self.uic.text_name.setText(name)
    
    @pyqtSlot(int, int, int, int, int)    
    def show_object_flag(self, bistro, cungdinh, haohao, kokomi, omachi):
        flag.bistro     = bistro
        flag.cungdinh   = cungdinh
        flag.haohao     = haohao
        flag.kokomi     = kokomi
        flag.omachi     = omachi
        flag.flag_setName = [bistro, cungdinh, haohao, kokomi, omachi]
        # Display number of object
        self.uic.num_kokomi.setText(str(CountObject.kokomi))
        self.uic.num_cungdinh.setText(str(CountObject.cung_dinh))
        self.uic.num_haohao.setText(str(CountObject.hao_hao))
        self.uic.num_omachi.setText(str(CountObject.omachi))
        self.uic.num_bistro.setText(str(CountObject.bistro))
        
    def auto_program(self):
        pos_pick = [220 *1000, -95 *1000, -121*1000, -180*10000, 0, 0]     
        
        if self.robot_status == True:
            if flag.cungdinh == 1:
                print("write byte 1")
                pos_pick[0] += int(((CenterObject.cung_dinh[1] -172)/2) * 1000)         # Tọa độ X
                pos_place = [-95*1000, -311*1000, -120*1000, -180*10000, 0, -88*10000]
                pos_place[2] += CountObject.cung_dinh * 4000                            # Tọa độ Z
                pos_place[5] -= CountObject.angle * 10000                               # Tọa độ Yaw
                device.writeVariablePos(121, pos_pick)
                device.writeVariablePos(101, pos_place)
                device.writeByte(21,1)
                CountObject.cung_dinh += 1

            elif flag.haohao == 1:
                print("write byte 2")
                pos_pick[0] +=  int(((CenterObject.hao_hao[1] -172)/2) * 1000)
                pos_place = [-45*1000, -220*1000, -120*1000, -180*10000, 0, -88*10000]
                pos_place[2] += CountObject.hao_hao * 4000
                pos_place[5] -= CountObject.angle * 10000
                device.writeVariablePos(121, pos_pick)
                device.writeVariablePos(102, pos_place)
                device.writeByte(21,2)
                CountObject.hao_hao += 1

            elif flag.kokomi == 1:
                print("write byte 3")
                pos_pick[0] += int(((CenterObject.kokomi[1] -172)/2) * 1000)
                pos_place = [-140*1000, -222*1000, -120*1000, -180*10000, 0, -88*10000]
                pos_place[2] +=  CountObject.kokomi * 4000   
                pos_place[5] -= CountObject.angle * 10000
                device.writeVariablePos(121, pos_pick)
                device.writeVariablePos(103, pos_place)
                device.writeByte(21,3)
                CountObject.kokomi += 1

            elif flag.bistro == 1:
                print("write byte 4")
                pos_pick[0] += int(((CenterObject.bistro[1] -172)/2) * 1000)
                pos_place = [ 7 *1000, -310*1000, -120*1000, -180*10000, 0, -88*10000]
                pos_place[2] +=  CountObject.bistro * 4000
                pos_place[5] -= CountObject.angle * 10000
                device.writeVariablePos(121, pos_pick)
                device.writeVariablePos(104, pos_place)
                device.writeByte(21,4)
                CountObject.bistro += 1

            elif flag.omachi == 1:
                print("write byte 5")
                pos_pick[0] += int(((CenterObject.omachi[1] -172)/2) * 1000)
                pos_place = [ 50 *1000, -216*1000, -120*1000, -180*10000, 0, -88*10000]
                pos_place[2] +=  CountObject.omachi * 4000
                pos_place[5] -= CountObject.angle * 10000
                device.writeVariablePos(121, pos_pick)
                device.writeVariablePos(105, pos_place)
                device.writeByte(21,5)
                CountObject.omachi += 1
                    
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
    
    @checkConnect
    @checkServo
    def TEACH_POINT(self):
        if self.point_count_1 >= self.uic.Point_teach_1.rowCount():
            self.add_row_table_1()
                 
        item = QTableWidgetItem()
        item.setText(self.uic.txt_pos.text())
        item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.uic.Point_teach_1.setItem(self.point_count_1, tableColumn.STT.value, item)
        
        item = QTableWidgetItem()
        item.setText(str(self.uic.S_move_text.text()))
        item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.uic.Point_teach_1.setItem(self.point_count_1, tableColumn.X_col.value, item)
        item = QTableWidgetItem()
        item.setText(str(self.uic.L_move_text.text()))
        item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.uic.Point_teach_1.setItem(self.point_count_1, tableColumn.Y_col.value, item)
        item = QTableWidgetItem()
        item.setText(str(self.uic.U_move_text.text()))
        item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.uic.Point_teach_1.setItem(self.point_count_1, tableColumn.Z_col.value, item)
        item = QTableWidgetItem()
        item.setText(str(self.uic.R_move_text.text()))
        item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.uic.Point_teach_1.setItem(self.point_count_1, tableColumn.Roll_col.value, item)
        item = QTableWidgetItem()
        item.setText(str(self.uic.B_move_text.text()))
        item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.uic.Point_teach_1.setItem(self.point_count_1, tableColumn.Pitch_col.value, item)
        item = QTableWidgetItem()
        item.setText(str(self.uic.T_move_text.text()))
        item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.uic.Point_teach_1.setItem(self.point_count_1, tableColumn.Yaw_col.value, item)

        self.point_count_1 += 1
        self.STT_count_1 += 1
    
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
            
    # MANUAL
    @checkConnect
    def CONVEYOR(self):
        if self.conveyor_status == False:
            device.writeByte(9, 1)
            self.uic.btn_conveyor.setText("OFF")
            self.uic.lb_conveyor_status.setText("ON")
            self.uic.btn_conveyor.setStyleSheet("QPushButton {color: red;}")
            self.uic.lb_on_conveyor.show()
            self.uic.lb_off_conveyor.hide()
            self.conveyor_status = True
        else:
            device.writeByte(9, 0)
            self.uic.btn_conveyor.setText("ON")
            self.uic.lb_conveyor_status.setText("OFF")
            self.uic.btn_conveyor.setStyleSheet("QPushButton {color: green;}")
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
                self.error_msg("Servo is off\n"+"Check the connection !")
        else:
            self.error_msg("No connection with robot\n"+"Check the connection !")
              
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
            
    def error_msg(self, text):
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Icon.Warning)
        msg.setText(text)
        msg.setWindowTitle("WARNING")
        msg.setStandardButtons(QMessageBox.StandardButton.Ok | QMessageBox.StandardButton.Close)
        msg.exec_()