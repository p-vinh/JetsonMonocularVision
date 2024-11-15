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

        # OpenCV video capture
        self.video_input = cv2.VideoCapture(self.video_input_id)
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
        self.output = cv2.VideoWriter(self.output_path, fourcc, 21.0, (800, 800))

        self.alive = True
        self.thread = Thread(target=self.loop, args=())
        self.thread.daemon = True
        self.thread.start()

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
                self.img = self.pre_process(img_local)
                self.allow_read = True

                self.output.write(img_local)
        except Exception as e:
            print("Error: ", e)

    def pre_process(self, img):
        # Resize the image to 800x800
        img_resized = cv2.resize(img, (800, 800))
        return img_resized
    
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