#!/usr/bin/python3

import time
import atexit
from PIL import Image
from sahi.predict import get_sliced_prediction
from sahi import AutoDetectionModel
from threading import Thread
import numpy
from RecordingInput import RecordingInput
from sahi.utils.cv import visualize_prediction
import numpy as np

class StrawberryDetector:
    def __init__(self, camera, log_file):
        self.camera = camera
        self.d_net = AutoDetectionModel.from_pretrained(
        	model_type="yolov8",
        	model_path="../model/weights.pt",
        	confidence_threshold=0.4,
        	device="cuda:0"
        )
        self.counter = 0
        self.log_f = log_file
        self.isRunning = True
        self.thread = Thread(target=self.thread_loop, args=())
        self.thread.deamon = True
        time.sleep(2.0)
        self.thread.start()

    def thread_loop(self):
        while self.isRunning:
            image = None
            image_np = None
            detections = None
            
            '''
            # Get images from both cameras.
            while not self.camera.allow_read:
                pass
            jetson_image = self.camera.img
            '''
            image_np = self.camera.get_frame()
            while image_np is None:
                image_np = self.camera.get_frame()
            image = Image.fromarray(image_np)
            
            try:
                detections = get_sliced_prediction(
                	image=image,
                	detection_model=self.d_net,
                	postprocess_class_agnostic=True,
                    slice_height=360,
                    slice_width=480,
                	overlap_height_ratio=0.2,
                	overlap_width_ratio=0.2
                )
                detections.export_visuals(export_dir="Images", file_name=str(self.counter))
                bounding_boxes = []
                class_ids = []
                
                for detections in detections.object_prediction_list:
                    bb = detections.bbox
                    coords = [bb.minx, bb.miny, bb.maxx, bb.maxy]
                    bounding_boxes.append(coords)
                    class_ids.append(detections.category.id)

                annotated = visualize_prediction(
                    image=image_np, 
                    boxes= bounding_boxes,
                    classes= class_ids
                )
                self.camera.save_frame(numpy.ascontiguousarray(annotated["image"]))
                print("Image number " + str(self.counter) + " exported")
                self.counter += 1
            except Exception as e:
                print("Error in detection: " + str(e.with_traceback()))

    def kill(self):
        self.isRunning = False
        self.camera.kill()

def kill(core):
    core.kill()

if __name__ == '__main__':
    log_file = open("Detect_last.log", "w")
    # camera = Jetson_Camera("0", "./Recordings", "Strawberry_Health", False)
    camera = RecordingInput("./Recordings", "left_camera.avi", "./Recordings")
    detector = StrawberryDetector(camera, log_file)
    atexit.register(kill, detector)
