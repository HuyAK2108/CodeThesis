from PyQt5 import QtGui
from PyQt5.QtWidgets import QMainWindow, QApplication, QLabel, QVBoxLayout
from PyQt5.QtGui import QPixmap
import sys
import cv2
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread
import numpy as np
from gui import Ui_MainWindow

class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()
        self._run_flag = True
        self.no_signal = cv2.imread("no_signal.jpg")
        
    def run(self):
        # capture from web cam
        cap = cv2.VideoCapture(0)
        while self._run_flag:
            ret, cv_img = cap.read()
            if ret:
                self.change_pixmap_signal.emit(cv_img)
        # shut down capture system
        cap.release()   
        self.change_pixmap_signal.emit(self.no_signal)

    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        self._run_flag = False
        self.wait()


class MainWindow (QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Qt live label demo")
        self.uic = Ui_MainWindow()
        self.uic.setupUi(self)
        self.uic.btn_onCAM.clicked.connect(self.start_capture_video)
        self.uic.btn_offCAM.clicked.connect(self.stop_capture_video)
        
    def start_capture_video(self):
        # create the video capture thread
        self.thread = VideoThread()
        # connect its signal to the update_image slot
        self.thread.change_pixmap_signal.connect(self.update_image)
        # start the thread
        self.thread.start()

    def stop_capture_video(self):
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
    
if __name__=="__main__":
    app = QApplication(sys.argv)
    a = MainWindow()
    a.show()
    sys.exit(app.exec_())
