# Module to Process

from time import time
import cv2
import numpy as np
import torch
from PyQt5.QtCore import Qt
from PyQt5 import QtGui
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QPixmap

model = torch.hub.load('D:/Python/Senior/yolov5','custom', path = 'D:/Python/Senior/yolov5/v5.pt', source= 'local')

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
            result = model(cv_img)
            print(result)
            df = result.pandas().xyxy[0]
            print(df)

            for ind in df.index:
                x1, y1 = int(df['xmin'][ind]), int(df['ymin'][ind])
                x2, y2 = int(df['xmax'][ind]), int(df['ymax'][ind])
                label = df['name'][ind]
                conf = df['confidence'][ind]
                text = label + ' ' + str(conf.round(decimals= 2))
                cv2.rectangle(cv_img, (x1, y1), (x2, y2), (255, 255, 0), 2)
                cv2.putText(cv_img, text, (x1, y1 - 5), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 0), 2)
            if ret:
                self.change_pixmap_signal.emit(cv_img)
               
        # shut down capture system
        cap.release()   
        self.change_pixmap_signal.emit(self.no_signal)

    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        self._run_flag = False
        self.wait()