#!/usr/bin/env python
import os
import atexit
from datetime import datetime
import numpy as np
import cv2 as cv

class RecordingInput:

    def __init__(self, recording_dir, recording_name, output_dir, output_name=None):
        self.video_input = cv.VideoCapture(recording_dir+ '/' + recording_name)

        if output_name is None:
            now = datetime.now()
            output_name = now.strftime("%Y-%m-%d_%H_%M_%S")
        fourcc = cv.VideoWriter_fourcc(*'XVID')
        self.output = cv.VideoWriter(output_dir + '/' + output_name + '.avi' , fourcc, 25.0, (1920, 1080))
        atexit.register(self.kill)
    
    def get_frame(self):
        ret, frame = self.video_input.read()
        return frame
    
    def save_frame(self, img):
        self.output.write(img)
    
    def kill(self):
        self.video_input.release()
        self.output.release()
        cv.destroyAllWindows()
            
