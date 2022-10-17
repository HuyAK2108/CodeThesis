# Use this file to control GUI
from PyQt5 import QtGui, QtSerialPort, QtCore
from PyQt5.QtSerialPort import QSerialPortInfo
from PyQt5.QtWidgets import QMainWindow, QApplication, QAction, QPlainTextEdit
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import  pyqtSlot, Qt, pyqtSignal, QTimer
import sys
import cv2, serial
import numpy as np
from gui import Ui_MainWindow
from module import VideoThread
from robot_controller import Robot, GetPosition

class MainWindow (QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Qt live label demo")
        self.uic = Ui_MainWindow()
        self.uic.setupUi(self)
        
        self.thread = {}
        
        # Button ON - OFF camera
        self.uic.btn_onCAM.clicked.connect(self.start_capture_video)
        self.uic.btn_offCAM.clicked.connect(self.stop_capture_video)
        # Button OPEN - CLOSE uart
        self.uic.btn_setuart.clicked.connect(self.connect_Serial)
        self.uic.btn_closeuart.clicked.connect(self.disconnect_Serial)
        # Button CONNECT - DISCONNECT robot
        self.uic.btn_setconnnect.clicked.connect(self.connect_robot)
        
    def start_capture_video(self):
        """Start video object detection and turn on camera
        """
        self.uic.btn_onCAM.setEnabled(False)
        self.uic.btn_offCAM.setEnabled(True)
        # create the video capture thread
        self.thread[1] = VideoThread(index=1)
        # connect its signal to the show_info slot to display object name
        self.thread[1].signal.connect(self.show_info)
        # connect its signal to the show_number slot to display object number
        self.thread[1].number.connect(self.show_number)
        self.thread[1].number2.connect(self.show_number2)
        self.thread[1].number3.connect(self.show_number3)
        self.thread[1].number4.connect(self.show_number4)
        self.thread[1].number5.connect(self.show_number5)
        # connect its signal to the update_image slot to display webcam
        self.thread[1].change_pixmap_signal.connect(self.update_image)
        # start the thread
        self.thread[1].start()
        
    def stop_capture_video(self):
        """Stop capture video and turn off camera
        """
        self.uic.btn_onCAM.setEnabled(True)
        self.uic.btn_offCAM.setEnabled(False)
        self.thread[1].stop()
    
    def closeEvent(self, event):
        """Close window
        """
        self.thread[1].stop()
        event.accept()
        
    def connect_Serial(self):
        """Open UART to send/receive data
        """
        self.com = self.uic.COM.currentText()
        self.baudrate = int(self.uic.Baudrate.currentText())
        self.uic.btn_closeuart.setEnabled(True)
        self.uic.btn_setuart.setEnabled(False)
        print("Connected to",self.com)    
        print("Baudrate",self.baudrate)  
        
    def disconnect_Serial(self):
        """Close UART
        """
        self.com = self.uic.COM.currentText()        
        self.uic.btn_closeuart.setEnabled(False)
        self.uic.btn_setuart.setEnabled(True)
        print("Disconnected to",self.com)    
    
    def connect_robot(self):
        self.thread[2] = Robot(index=2)
        self.thread[2].start()
            
    
    
        """Transmit data between classes
        """
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
    @pyqtSlot(str)
    def show_number(self, gaudo):
        self.uic.num_gaudo.setText(gaudo)
    @pyqtSlot(str)
    def show_number2(self, cungdinh):
        self.uic.num_cungdinh.setText(cungdinh)
    @pyqtSlot(str)
    def show_number3(self, haohao):
        self.uic.num_haohao.setText(haohao)
    @pyqtSlot(str)
    def show_number4(self, omachi102):
        self.uic.num_omachi102.setText(omachi102)
    @pyqtSlot(str)
    def show_number5(self, omachispa):
        self.uic.num_omachispa.setText(omachispa)
        
# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     a = MainWindow()
#     a.show()
#     sys.exit(app.exec_())
