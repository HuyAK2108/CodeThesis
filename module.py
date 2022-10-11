# Module to Process

from time import time
import cv2
import numpy as np
import torch
from PyQt5.QtCore import QThread, pyqtSignal
from tracker import CentroidTracker

tracker = CentroidTracker()
# Load model
model = torch.hub.load('D:/Python/Senior/yolov5','custom', path = 'D:/Python/Senior/yolov5/v5.pt', source= 'local')
device = 'cuda' if torch.cuda.is_available() else 'cpu'
clasess = model.names
# classes = ['cung dinh', 'gau do', 'hao hao', 'omachi 102', 'omachi spaghetti']

class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)
    signal = pyqtSignal(str)
    
    def __init__(self):
        super(VideoThread, self).__init__()
        
        # Live stream parameters
        self._run_flag = True
        self.no_signal = cv2.imread("no_signal.jpg")

    def run(self):
        """
        Initializes the model, classes and device
        """
        # self.model = self.load_model()  # Load model 
        self.run_program()

    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        self._run_flag = False
        self.wait()
    
    def score_frame(self, frame):
        """Takes a single frame as input and scores the frame using yolov5 model

        Args:
            frame (): input frame in numpy/list/tuple format

        Returns:
             Labels and Coordinates of objects detected by model in the frame.
        """
        # a = torch.Tensor.cpu()
        model.to(self.device)
        frame = [frame]
        results = model(frame)
        labels = results.pandas().xyxy[0]
        cord = results.pandas().xyxy[0]
        return labels, cord
    
    def plot_boxes(self, results, frame):
        """Takes a frame and its results as input, and plots the bounding boxes and label on to the frame.

        Args:
            results: contains labels and coordinates predicted by model on the given frame.
            frame: Frame which has been scored

        Returns:
            Frame with bouding boxes and labels ploted on it 
        """
        # self.object_name = ''
        results = model(frame)
        # print(result)         
        df = results.pandas().xyxy[0]
        rects = []                              
        detections = []
        # print(df)
        for ind in df.index:
            label = df['name'][ind]
            conf = df['confidence'][ind]
            if conf > 0.9:
                x1, y1 = int(df['xmin'][ind]), int(df['ymin'][ind])
                x2, y2 = int(df['xmax'][ind]), int(df['ymax'][ind])
                text = label + ' ' + str(conf.round(decimals= 2))
                detections.append([x1, y1, x2 - x1, y2 - y1])
                rects.append([x1, y1, x2, y2])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame, text, (x1, y1 - 5), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)
                # cv2.putText(frame, text, (x1, y1 - 5), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 255), 2)
                # self.object_name = label
                self.signal.emit(label)     
        rects_ids = tracker.update(rects)
        for objectID, centroid in rects_ids.items():
            # print("str(objectID)", objectID) # 1 - số lượng object
            # print("centroid", centroid)      # [X_center, Y_center] - tọa độ điểm giữa của object
            cv2.putText(frame, str(objectID), centroid, cv2.FONT_HERSHEY_SIMPLEX, 3, (128,255,255), 2)
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
            # objectname = self.label 
            end_time = time()
            self.fps = 1 / (np.round(end_time - start_time, 3))
            text = round(self.fps,2)
            cv2.putText(cv_img,"FPS: {}".format(str(text)), (10, 40), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 255), 2) # write FPS on bbox
            # print(f"Frames Per Second : {round(self.fps,2)} FPS")
            if ret:
                self.change_pixmap_signal.emit(cv_img)
        # shut down capture system
        cap.release()   
        self.change_pixmap_signal.emit(self.no_signal) 