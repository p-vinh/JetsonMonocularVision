import os
from datetime import datetime
from threading import Thread
import ultralytics
import supervision as sv
import numpy as np
import cv2


class Jetson_Camera:

    def __init__(self, input_num, recording_dir, recording_name, flip=False):
        self.img = None
        self.video_input = None
        self.allow_read = True

        self.video_input_id = input_num
        self.flip = flip

        # Use Image as a test image
        self.video_input = cv2.imread("test_images/test.jpg") # Image Size of the original image 4000x3000

        if self.video_input is None:
            print(f"Error: Could not open video input {self.video_input_id}")
            return

        self.alive = True
        self.thread = Thread(target=self.loop, args=())
        self.thread.daemon = True
        self.thread.start()

    # Only slice the image to 640x480 from the left camera/ keep the original image from the right camera
    def loop(self):
        try:
            while self.alive:
                ret, img_local = self.video_input.read()
                if not ret:
                    print("Error: No image captured.")
                    continue

                if self.flip:
                    img_local = cv2.rotate(img_local, cv2.ROTATE_180)

                self.allow_read = False
                
                # Left camera will be used as a main camera. Right camera will be used as a reference camera.
                if self.video_input_id == 0:
                    self.img = self.pre_process(img_local)
                else: # Right camera
                    self.img = img_local
                self.allow_read = True

        except Exception as e:
            print("Error: ", e)    

    def pre_process(self, img):
        # Use supervision's inference slicer to cut the image into 480x640
        slicer = sv.InferenceSlicer(slice_size=(480, 640))
        slices = slicer.slice_image(img)
        return slices
    
    def is_thread_alive(self):
        return self.thread.is_alive()

    def kill(self):
        print("Killing the camera")
        self.alive = False
        self.thread.join()
        self.video_input.release()
        cv2.destroyAllWindows()

# Example usage
if __name__ == "__main__":
    camera = Jetson_Camera(input_num=0, recording_dir=".", recording_name="test", flip=False) # not going to be used for testing
    try:
        while camera.is_thread_alive():
            pass
    except KeyboardInterrupt:
        camera.kill()