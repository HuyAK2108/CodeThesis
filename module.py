from time import time
import cv2, os
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
model = torch.hub.load("D:/Python/yolov5-master",'custom', path = 'model/Dec14th.pt', source= 'local')
# device = 'cuda' if torch.cuda.is_available() else 'cpu'
# model.to(device)
clasess = model.names
print(clasess)
class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)
    label_signal  = pyqtSignal(str)
    flag_signal   = pyqtSignal(int, int, int, int, int)
    
    def __init__(self, index = 0):
        super(VideoThread, self).__init__()
        self.index = index
        print("Video thread start", self.index)
        # Live stream parameters
        self._run_flag = True
        self.no_signal = cv2.imread("pictures/no_signal.jpg")
        num = len(os.listdir("videos"))
        self.output = f"videos/report_{num+1}.mp4"
        self.kernel = np.ones((20, 20), np.uint8)
        # Object parameters
        self.object_name = ''
        self.start_point = [400, 0]
        self.end_point   = [400, 480]
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
        
        centroid_1 = [0,0]
        centroid_2 = [0,0]
        centroid_3 = [0,0]
        centroid_4 = [0,0]
        centroid_5 = [0,0]
        # print(df)
        for ind in df.index:
            label = df['name'][ind]
            conf = df['confidence'][ind]
            if conf > 0.7:
                x1, y1 = int(df['xmin'][ind]), int(df['ymin'][ind])
                x2, y2 = int(df['xmax'][ind]), int(df['ymax'][ind])
                text = label + ' ' + str(conf.round(decimals= 2))
                # detections.append([x1, y1, x2 - x1, y2 - y1])
                rects.append([x1, y1, x2, y2])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame, text, (x1, y1 - 5), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)
                self.label_signal.emit(label)  
                self.object_name = label
        
        """Bistro
        """
        if self.object_name == 'BISTRO':
            rects_ids = tracker.update(rects)
            for _ , centroid_1 in rects_ids.items():
                cv2.circle(frame, centroid_1, 5, (0,0,255), -1)
                CenterObject.bistro = centroid_1    
                    
        """Cung dinh
        """
        if self.object_name == 'CUNG DINH':
            rects_ids_2 = tracker_2.update(rects)
            for _, centroid_2 in rects_ids_2.items():
                cv2.circle(frame, centroid_2, 5, (0,0,255), -1)
                CenterObject.cung_dinh = centroid_2
        
        """Hao hao
        """
        if self.object_name == 'HAO HAO':
            rects_ids_3 = tracker_3.update(rects)
            for _, centroid_3 in rects_ids_3.items():
                cv2.circle(frame, centroid_3, 5, (0,0,255), -1)
                CenterObject.hao_hao = centroid_3

        """Kokomi
        """
        if self.object_name == 'KOKOMI':
            rects_ids_4 = tracker_4.update(rects)
            for _, centroid_4 in rects_ids_4.items():
                cv2.circle(frame, centroid_4, 5, (0,0,255), -1)
                CenterObject.kokomi = centroid_4
                
        """Omachi
        """
        if self.object_name == 'OMACHI':
            rects_ids_5 = tracker_5.update(rects)
            for _, centroid_5 in rects_ids_5.items():
                cv2.circle(frame, centroid_5, 5, (0,0,255), -1)
                CenterObject.omachi = centroid_5
        
        if centroid_1[0] >= self.start_point[0] - constVariable.error_trigger and centroid_1[0]<=self.start_point[0] + constVariable.error_trigger:
            self.flag_1 = 1
        else:
            self.flag_1 = 0
            
        if centroid_2[0] >= self.start_point[0] - constVariable.error_trigger and centroid_2[0]<=self.start_point[0] + constVariable.error_trigger:
            self.flag_2 = 1
        else:
            self.flag_2 = 0
            
        if centroid_3[0] >= self.start_point[0] - constVariable.error_trigger and centroid_3[0]<=self.start_point[0] + constVariable.error_trigger:
            self.flag_3 = 1
        else:
            self.flag_3 = 0
            
        if centroid_4[0] >= self.start_point[0] - constVariable.error_trigger and centroid_4[0]<=self.start_point[0] + constVariable.error_trigger:
            self.flag_4 = 1
        else:
            self.flag_4 = 0
        
        if centroid_5[0] >= self.start_point[0] - constVariable.error_trigger and centroid_5[0]<=self.start_point[0] + constVariable.error_trigger:
            self.flag_5 = 1
        else:
            self.flag_5 = 0    
            
        self.flag_signal.emit(self.flag_1, self.flag_2, self.flag_3, self.flag_4, self.flag_5)
        return frame
    
    def run_program(self):
        """This function runs the loop to read the video frame by frame 
        """
        # Capture from web cam
        cap = cv2.VideoCapture(0)
        
        _, bg = cap.read()
        bg_gray = cv2.cvtColor(bg, cv2.COLOR_BGR2GRAY)
        bg_gray = cv2.GaussianBlur(bg_gray, (7,7), 0)    
                      
        # Save video capture from webcam
        out_video = cv2.VideoWriter(self.output, cv2.VideoWriter_fourcc(*'mp4v'), 15, (640, 480))
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
            out_video.write(cv_img) # Write video
            # Background substraction 
            _, frame = cap.read()
            roi = frame
            gray_frame = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            gray_frame = cv2.GaussianBlur(gray_frame, (7,7), 0)

            difference = cv2.absdiff(bg_gray, gray_frame)
            
            _, difference = cv2.threshold(difference, 20, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            img_dilation = cv2.dilate(difference, self.kernel, iterations= 2)
            img_erosion = cv2.erode(img_dilation, self.kernel, iterations= 2)
            
            # cnts, hierachy = cv2.findContours(difference, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
            cnts, hierachy = cv2.findContours(img_erosion, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
              
            for _, c in enumerate(cnts):
                area = cv2.contourArea(c)
                if area < 5000 or area > 30000:
                    continue
                rect = cv2.minAreaRect(c)
                box  = cv2.boxPoints(rect)
                box  = np.int0(box)

                center = (int(rect[0][0]),int(rect[0][1])) 
                width = int(rect[1][0])
                height = int(rect[1][1])
                angle = int(rect[2])
                
                if width < height:
                  angle = 90 - angle
                else:
                  angle = -angle
                  
                CountObject.angle = angle
                cv2.putText(roi, "Angle: {}".format(str(angle)), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2, cv2.LINE_AA)
                cv2.drawContours(roi,[box],0,(0,0,255),2)

            if flag.trigger == True:
                cv2.line(cv_img, self.start_point, self.end_point, (0, 255, 0), 2)  # Trigger
                
            if flag.conveyor == True:
                cv2.line(cv_img, (0,120), (640,120), (0, 0, 255), 2)                # Canh chỉnh camera
                cv2.line(cv_img, (0,320), (640,320), (0, 0, 255), 2)                # Canh chỉnh camera
            
            if ret:
                if flag.bgs == True:    
                    self.change_pixmap_signal.emit(roi)
                elif flag.bw == True:
                    # self.change_pixmap_signal.emit(difference)
                    self.change_pixmap_signal.emit(img_erosion)
                else:
                    self.change_pixmap_signal.emit(cv_img)
        # shut down capture system
        cap.release()   
        out_video.release()
        self.change_pixmap_signal.emit(self.no_signal) 
        print("Video thread stop", self.index)