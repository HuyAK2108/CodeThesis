# Use this file to control GUI
from PyQt5 import QtGui, QtSerialPort, QtCore
from PyQt5.QtSerialPort import QSerialPortInfo
from PyQt5.QtWidgets import QMainWindow, QApplication, QAction, QPlainTextEdit
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import  pyqtSlot, Qt, pyqtSignal, QTimer
import sys
import cv2
import numpy as np
from gui import Ui_MainWindow
from module import VideoThread

class MainWindow (QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Qt live label demo")
        self.uic = Ui_MainWindow()
        self.uic.setupUi(self)
        self.timer = QTimer()
        self.uic.btn_onCAM.clicked.connect(self.start_capture_video)
        self.uic.btn_offCAM.clicked.connect(self.stop_capture_video)
        
    def start_capture_video(self):
        self.uic.btn_onCAM.setEnabled(False)
        self.uic.btn_offCAM.setEnabled(True)
        # create the video capture thread
        self.thread = VideoThread()
        # connect its signal to the show_info slot to display object name
        self.thread.signal.connect(self.show_info)
        # connect its signal to the show_number slot to display object number
        self.thread.number.connect(self.show_number)
        self.thread.number2.connect(self.show_number2)
        self.thread.number3.connect(self.show_number3)
        self.thread.number4.connect(self.show_number4)
        self.thread.number5.connect(self.show_number5)
        # connect its signal to the update_image slot to display webcam
        self.thread.change_pixmap_signal.connect(self.update_image)
        # start the thread
        self.thread.start()
        
    def stop_capture_video(self):
        self.uic.btn_onCAM.setEnabled(True)
        self.uic.btn_offCAM.setEnabled(False)
        self.thread.stop()
    
    def closeEvent(self, event):
        self.thread.stop()
        event.accept()
        
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
    
    @pyqtSlot(str) 
    def show_info(self, name):
        self.uic.text_name.setPlainText(name)
    
    @pyqtSlot(str)
    def show_number(self, gaudo):
        self.uic.num_gaudo.setPlainText(gaudo)
    @pyqtSlot(str)
    def show_number2(self, cungdinh):
        self.uic.num_cungdinh.setPlainText(cungdinh)
    @pyqtSlot(str)
    def show_number3(self, haohao):
        self.uic.num_haohao.setPlainText(haohao)
    @pyqtSlot(str)
    def show_number4(self, omachi102):
        self.uic.num_omachi102.setPlainText(omachi102)
    @pyqtSlot(str)
    def show_number5(self, omachispa):
        self.uic.num_omachispa.setPlainText(omachispa)
        
if __name__ == "__main__":
    app = QApplication(sys.argv)
    a = MainWindow()
    a.show()
    sys.exit(app.exec_())
