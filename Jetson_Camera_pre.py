#!/usr/bin/python
import os
from datetime import datetime
from threading import Thread
import cv2


class Jetson_Camera:

    def __init__(self, input_num, recording_dir, recording_name, flip=False):
        self.img = None
        self.video_input = None
        self.allow_read = True

        self.video_input_id = input_num
        self.flip = flip

        self.gstreamer_pipeline = (
            f"nvarguscamerasrc sensor-id={self.video_input_id} ! "
            "video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! "
            "nvvidconv flip-method=2 ! video/x-raw, width=800, height=800, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! appsink"
        )


        # OpenCV video capture
        self.video_input = cv2.VideoCapture(self.gstreamer_pipeline, cv2.CAP_GSTREAMER)
        if not self.video_input.isOpened():
            print(f"Error: Could not open video input {self.video_input_id}")
            return

        print(recording_name + " Camera input is: " + str(input_num))

        if recording_name is None:
            now = datetime.now()
            recording_name = now.strftime("%Y-%m-%d_%H_%M_%S")
        self.output_path = os.path.join(recording_dir, recording_name + '.avi')

        # OpenCV video writer
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        
        # if output path does not exist, create it
        if not os.path.exists(recording_dir):
            os.makedirs(recording_dir)
        self.output = cv2.VideoWriter(self.output_path, fourcc, 21.0, (800, 800))

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
                self.img = img_local
                self.output.write(img_local)
        except Exception as e:
            print("Error: ", e)
    
    def is_thread_alive(self):
        return self.thread.is_alive()

    def kill(self):
        print("Killing the camera")
        self.alive = False
        self.thread.join()
        self.video_input.release()
        self.output.release()
        cv2.destroyAllWindows()

# Example usage
if __name__ == "__main__":
    camera = Jetson_Camera(input_num=0, recording_dir=".", recording_name="test", flip=False)
    try:
        while camera.is_thread_alive():
            pass
    except KeyboardInterrupt:
        camera.kill()
