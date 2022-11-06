from time import time
import cv2
import numpy as np
import torch
from PyQt5.QtCore import QThread, pyqtSignal
from tracker import CentroidTracker, CentroidTracker2, CentroidTracker3, CentroidTracker4, CentroidTracker5
from typedef import *

tracker   = CentroidTracker()
tracker_2 = CentroidTracker2()
tracker_3 = CentroidTracker3()
tracker_4 = CentroidTracker4()
tracker_5 = CentroidTracker5()
# Load model
model = torch.hub.load('D:/Python/Senior/yolov5','custom', path = 'model/best.pt', source= 'local')
device = 'cuda' if torch.cuda.is_available() else 'cpu'
model.to(device)
clasess = model.names
print(clasess)
class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)
    signal  = pyqtSignal(str)
    number  = pyqtSignal(str, str, str, str, str)
    position = pyqtSignal(int, int, int, int, int)
    
    def __init__(self, index = 0):
        super(VideoThread, self).__init__()
        self.index = index
        print("Video thread start", self.index)
        # Live stream parameters
        self._run_flag = True
        self.no_signal = cv2.imread("pictures/no_signal.jpg")
        
        # Object parameters
        self.object_name = ''
        self.count_kokomi       = 0
        self.count_haohao       = 0 
        self.count_omachi       = 0
        self.count_cungdinh     = 0 
        self.count_miliket      = 0 
        self.start_point = [500, 0]
        self.end_point   = [500, 480]
        self.flag_1 = 0
        self.flag_2 = 0
        self.flag_3 = 0
        self.flag_4 = 0
        self.flag_5 = 0

    def run(self):
        """
        Initializes the model, classes and device
        """
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
        results = model(frame)
        # print(result)         
        df = results.pandas().xyxy[0]
        rects = []                              
        detections      = []
        detections_2    = []
        detections_3    = []
        detections_4    = []
        detections_5    = []
        centroid_1 = [0,0]
        centroid_2 = [0,0]
        centroid_3 = [0,0]
        centroid_4 = [0,0]
        centroid_5 = [0,0]

        # print(df)
        for ind in df.index:
            label = df['name'][ind]
            conf = df['confidence'][ind]
            if conf > 0.8:
                x1, y1 = int(df['xmin'][ind]), int(df['ymin'][ind])
                x2, y2 = int(df['xmax'][ind]), int(df['ymax'][ind])
                text = label + ' ' + str(conf.round(decimals= 2))
                # detections.append([x1, y1, x2 - x1, y2 - y1])
                rects.append([x1, y1, x2, y2])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame, text, (x1, y1 - 5), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)
                # cv2.putText(frame, text, (x1, y1 - 5), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 255), 2)
                
                self.signal.emit(label)  
                self.object_name = label
        
        """Kokomi
        """
        if self.object_name == 'Kokomi':
            rects_ids = tracker.update(rects)
            for objectID, centroid_1 in rects_ids.items():
                detections.append(objectID)
                self.count_kokomi = get_lastest_value(detections)
                cv2.putText(frame, str(self.count_kokomi), centroid_1, cv2.FONT_HERSHEY_SIMPLEX, 3, (128,255,255), 2)
        
        """Cung dinh
        """
        if self.object_name == 'Cung dinh':
            rects_ids = tracker_2.update(rects)
            for objectID_2, centroid_2 in rects_ids.items():
                detections_2.append(objectID_2)
                self.count_cungdinh = get_lastest_value(detections_2)
                cv2.putText(frame, str(self.count_cungdinh), centroid_2, cv2.FONT_HERSHEY_SIMPLEX, 3, (128,255,255), 2)
        
        """Hao hao
        """
        if self.object_name == 'Hao Hao':
            rects_ids = tracker_3.update(rects)
            for objectID_3, centroid_3 in rects_ids.items():
                detections_3.append(objectID_3)
                self.count_haohao = get_lastest_value(detections_3)
                cv2.putText(frame, str(self.count_haohao), centroid_3, cv2.FONT_HERSHEY_SIMPLEX, 3, (128,255,255), 2)
                # print("detection:",detections_3)
            print("center point:",centroid_3[0])

        """Omachi
        """
        if self.object_name == 'Omachi':
            rects_ids = tracker_4.update(rects)
            for objectID_4, centroid_4 in rects_ids.items():
                detections_4.append(objectID_4)
                self.count_omachi = get_lastest_value(detections_4)
                cv2.putText(frame, str(self.count_omachi), centroid_4, cv2.FONT_HERSHEY_SIMPLEX, 3, (128,255,255), 2)
        
        """Miliket
        """
        if self.object_name == 'Miliket':
            rects_ids = tracker_5.update(rects)
            for objectID_5, centroid_5 in rects_ids.items():
                detections_5.append(objectID_5)
                self.count_miliket = get_lastest_value(detections_5)
                cv2.putText(frame, str(self.count_miliket), centroid_5, cv2.FONT_HERSHEY_SIMPLEX, 3, (128,255,255), 2)          

        self.number.emit(str(self.count_kokomi), str(self.count_cungdinh), str(self.count_haohao), str(self.count_omachi), str(self.count_miliket))         
        if centroid_1[0] >= self.start_point[0]-10 and centroid_1[0]<=self.start_point[0]+10:
            self.flag_1 = 1
        else:
            self.flag_1 = 0
            
        if centroid_2[0] >= self.start_point[0]-10 and centroid_2[0]<=self.start_point[0]+10:
            self.flag_2 = 1
        else:
            self.flag_2 = 0
            
        if centroid_3[0] >= self.start_point[0]-10 and centroid_3[0]<=self.start_point[0]+10:
            self.flag_3 = 1
        else:
            self.flag_3 = 0
            
        if centroid_4[0] >= self.start_point[0]-10 and centroid_4[0]<=self.start_point[0]+10:
            self.flag_4 = 1
        else:
            self.flag_4 = 0
        
        if centroid_5[0] >= self.start_point[0]-10 and centroid_5[0]<=self.start_point[0]+10:
            self.flag_5 = 1
        else:
            self.flag_5 = 0    
            
        self.position.emit(self.flag_1, self.flag_2, self.flag_3, self.flag_4, self.flag_5)
        return frame
    
    def run_program(self):
        """This function runs the loop to read the video frame by frame 
        """
        # Capture from web cam
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FPS, 30)
        # Loop to read the video frame by frame
        while self._run_flag:
            start_time = time()            
            ret, cv_img = cap.read() 
            results = self.score_frame(cv_img)
            cv_img = self.plot_boxes(results, cv_img)
            end_time = time()
            self.fps = 1 / (np.round(end_time - start_time, 3))
            text = round(self.fps,2)
            cv2.putText(cv_img,"FPS: {}".format(str(text)), (10, 40), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 255), 2) # write FPS on bbox
            cv2.line(cv_img, self.start_point, self.end_point, (171, 97, 193), 2)
            
            # print(f"Frames Per Second : {round(self.fps,2)} FPS")
            if ret:
                self.change_pixmap_signal.emit(cv_img)
        # shut down capture system
        cap.release()   
        self.change_pixmap_signal.emit(self.no_signal) 
        print("Video thread stop", self.index)
        
def get_lastest_value(value):
    return max(value)