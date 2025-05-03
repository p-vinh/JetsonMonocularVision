#!/usr/bin/python3

import time
import atexit
from PIL import Image
from sahi.predict import get_sliced_prediction
from sahi import AutoDetectionModel
from threading import Thread
import numpy
from RecordingInput import RecordingInput


class StrawberryDetector:
    def __init__(self, camera, log_file):
        self.camera = camera
        self.d_net = AutoDetectionModel.from_pretrained(
        	model_type="yolo11",
        	model_path="../YOLOv11L_Strawberry/weights/best.pt",
        	confidence_threshold=0.4,
        	device="cuda:0"
        )
        self.counter = 0;
        self.log_f = log_file
        self.isRunning = True
        self.thread = Thread(target=self.thread_loop, args=())
        self.thread.deamon = True
        time.sleep(2.0)
        self.thread.start()

    def thread_loop(self):
        while self.isRunning:
            image = None
            detections = None
            
            '''
            # Get images from both cameras.
            while not self.camera.allow_read:
                pass
            jetson_image = self.camera.img
            '''
            image = self.camera.get_frame()
            while image is None:
                image = self.camera.get_frame()
            image = Image.fromarray(image)
            
            try:
                detections = get_sliced_prediction(
                	image=image,
                	detection_model=self.d_net,
                	postprocess_class_agnostic=True,
                	slice_height=640,
                	slice_width=640,
                	overlap_height_ratio=0.2,
                	overlap_width_ratio=0.2
                )
                detections.export_visuals(export_dir="Images", file_name=str(self.counter))
                # TODO This (line 64) add a frame to the video without annotations.
                #      Use 'visualize_prediction' from 'sahi.cv' to add annotations to the frame.
                self.camera.save_frame(numpy.ascontiguousarray(detections.image))
                print("Image number " + str(self.counter) + " exported")
                self.counter += 1;
            except Exception as e:
                print("Error: ", e)

    def kill(self):
        self.isRunning = False
        self.camera.kill()

def kill(core):
    core.kill()

if __name__ == '__main__':
    log_file = open("Strawberry_Health_log.log", "w")
    # camera = Jetson_Camera("0", "./Recordings", "Strawberry_Health", False)
    camera = RecordingInput("./Recordings", "left_camera.avi", "./Recordings")
    detector = StrawberryDetector(camera, log_file)
    atexit.register(kill, detector)
