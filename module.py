# Module to Process

from time import time
import cv2
import numpy as np
import torch
from PyQt5.QtCore import Qt
from PyQt5 import QtGui
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QPixmap

class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()
        self.device = None
        self.classes = None
        self._run_flag = True
        self.no_signal = cv2.imread("no_signal.jpg")

    def run(self):
        """
        Initializes the model, classes and device
        """
        self.model = self.load_model()  # Load model 
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.run_program()

    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        self._run_flag = False
        self.wait()
        
    def load_model(self):
        """Loads Yolov5 model and classes

        Returns:
            Trained Yolov5 model
        """
        self.model = torch.hub.load('D:/Python/Senior/yolov5','custom', path = 'D:/Python/Senior/yolov5/v5.pt', source= 'local')
        self.classes = ['cung dinh', 'gau do', 'hao hao', 'omachi 102', 'omachi spaghetti']
        return self.model
    
    def score_frame(self, frame):
        """Takes a single frame as input and scores the frame using yolov5 model

        Args:
            frame (): input frame in numpy/list/tuple format

        Returns:
             Labels and Coordinates of objects detected by model in the frame.
        """
        # a = torch.Tensor.cpu()
        self.model.to(self.device)
        frame = [frame]
        results = self.model(frame)
        # labels, cord = results.xyxyn[0][:, -1].numpy(), results.xyxyn[0][:, :-1].numpy()
        labels = results.pandas().xyxy[0]
        cord = results.pandas().xyxy[0]
        return labels, cord
    
    def class_to_label(self, x):
        """For a given label value, return the corresponding string value
            
        Args:
            x : numeric label - 0,1,2,3,4
            
        Returns:
            ['cung dinh', 'gau do', 'hao hao', 'omachi 102', 'omachi spaghetti']
        """
        return self.classes[int(x)]
    
    def plot_boxes(self, results, frame):
        """Takes a frame and its results as input, and plots the bounding boxes and label on to the frame.

        Args:
            results: contains labels and coordinates predicted by model on the given frame.
            frame: Frame which has been scored

        Returns:
            Frame with bouding boxes and labels ploted on it 
        """
        # labels, cord = results
        # n = len(labels)
        # x_shape, y_shape = frame.shape[1], frame.shape[0]
        # for i in range(n):
        #     row = cord[i]
        #     # print("ddd", round(cord[i][4], 2))
        #     if row[4] >= 0.2:
        #         x1, y1, x2, y2 = int(row[0] * x_shape), int(row[1] * y_shape), int(row[2] * x_shape), int(row[3] * y_shape)
        #         bgr = (0, 255, 0)
        #         cv2.rectangle(frame, (x1, y1), (x2, y2), bgr, 2)
        #         cv2.putText(frame, self.class_to_label(labels[i]) + " " + str(round(row[4], 2)), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, bgr, 2)

        results = self.model(frame)
        # print(result)         
        df = results.pandas().xyxy[0]
        # print(df)
        for ind in df.index:
            x1, y1 = int(df['xmin'][ind]), int(df['ymin'][ind])
            x2, y2 = int(df['xmax'][ind]), int(df['ymax'][ind])
            label = df['name'][ind]
            conf = df['confidence'][ind]
            text = label + ' ' + str(conf.round(decimals= 2))
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 2)
            cv2.putText(frame, text, (x1, y1 - 5), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 0), 2)
            # cv2.putText(frame, text, (x1, y1 - 5), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 255), 2)
        return frame
        
    
    def run_program(self):
        """This function runs the loop to read the video frame by frame 
        """
        # Capture from web cam
        cap = cv2.VideoCapture(0)
        
        # Loop to read the video frame by frame
        while self._run_flag:
            start_time = time()
            ret, cv_img = cap.read()
            
            results = self.score_frame(cv_img)
            cv_img = self.plot_boxes(results, cv_img)
            end_time = time()
            fps = 1 / (np.round(end_time - start_time, 3))
            print(f"Frames Per Second : {round(fps,2)} FPS")
            if ret:
                self.change_pixmap_signal.emit(cv_img)
            
            
            # result = self.model(cv_img)
            # # print(result)         
            # df = result.pandas().xyxy[0]
            # # print(df)

            # for ind in df.index:
            #     x1, y1 = int(df['xmin'][ind]), int(df['ymin'][ind])
            #     x2, y2 = int(df['xmax'][ind]), int(df['ymax'][ind])
            #     label = df['name'][ind]
            #     conf = df['confidence'][ind]
            #     text = label + ' ' + str(conf.round(decimals= 2))
            #     cv2.rectangle(cv_img, (x1, y1), (x2, y2), (255, 255, 0), 2)
            #     cv2.putText(cv_img, text, (x1, y1 - 5), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 0), 2)
            # if ret:
            #     self.change_pixmap_signal.emit(cv_img)
               
        # shut down capture system
        cap.release()   
        self.change_pixmap_signal.emit(self.no_signal)