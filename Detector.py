import atexit
import math
import sys
import time
from threading import Thread
import jetson_utils
import jetson_inference
from Jetson_Camera_2 import Jetson_Camera
from MavLink import MavLink
from Triangulation import stereo_vision


# TODO: Integrate triangulation, also add it as a parameter to self update.

class Detector:
    def __init__(self, left_input, right_input, left_save_name, right_save_name, dir_name, camera_pitch, GPS_node: MavLink,
                 flip=False):
        self.detection_timestamp = 0
        self.persons_cords = None
        self.average = None
        self.best_right = None
        self.people_on_the_right = None
        self.people_on_the_left = None
        self.best_left = None
        self.left_image = None
        self.right_image = None
        self.camera_offset = camera_pitch
        self.confidence_backlog = []
        self.location_backlog = []
        self.net = jetson_inference.detectNet()
        self.GPS = GPS_node
        self.logfile = open("Detector_last_log.log", "w")

        # print("Left input ID:", left_input)
        # print("Right input ID:", right_input)

        self.left_camera: Jetson_Camera = Jetson_Camera(str(left_input), dir_name, left_save_name, flip)
        self.right_camera: Jetson_Camera = Jetson_Camera(str(right_input), dir_name, right_save_name, flip)

        if flip:
            self.left_camera, self.right_camera = self.right_camera, self.left_camera

        self.isRunning = True
        self.allow_read = True

        self.thread = Thread(target=self.loop, args=())
        self.thread.deamon = True
        time.sleep(2.0)
        self.thread.start()

    def loop(self):
        average_of_confidences = 0.0
        persons_cords_local = None
        timestamp = 0
        while self.isRunning:
            while not self.left_camera.allow_read:
                pass
            left_image = self.left_camera.img

            while not self.right_camera.allow_read:
                pass
            right_image = self.right_camera.img

            while not self.GPS.allow_read:
                pass
            location_snapsave = self.GPS.loc

            left_detections = self.net.Detect(left_image, overlay="box,labels,conf")
            right_detections = self.net.Detect(right_image, overlay="box,labels,conf")

            best_left, people_left = self.get_best_detection(left_detections)
            best_right, people_right = self.get_best_detection(right_detections)

            if best_left is None or best_right is None:
                # print("Best right detection:", best_right)
                # print("Best left detection:", best_left)
                self.confidence_backlog = []
                average_of_confidences = 0.0
            else:
                # print("Best right detection:", best_right)
                # print("Best left detection:", best_left)

                # """" TEST ---
                print(best_left.Center)
                print(best_right.Center)
                distance, angle, vertical = stereo_vision(spread=0.8636,
                                                          center_right=best_right.Center,
                                                          center_left=best_left.Center,
                                                          fov=62.2,
                                                          image_width=left_image.width,
                                                          image_height=left_image.height)
                if not math.isnan(distance):
                    self.logfile.write("Test distance:" + str(distance) + "\n")
                    self.logfile.write("Test horizontal angle:" + str(angle) + "\n")
                    self.logfile.write("Test vertical angle:" + str(vertical) + "\n")
                    self.logfile.write("Test drone location: " + str(location_snapsave) + "\n")
                    persons_cords_local = self.GPS.get_persons_GPS(distance, angle, location_snapsave,
                                                                   vertical, self.camera_offset)
                    self.logfile.write("Test coordinates:" + str(persons_cords_local) + "\n\n")
                    # persons_cords_local = self.GPS.get_persons_GPS(distance, angle, location_snapsave)
                    print("DETECTOR:    Test: persons coordinates are:", self.persons_cords)
                # --- END TEST """

                self.confidence_backlog.append(max(best_right.Confidence, best_left.Confidence))
                # print("DETECTOR:  backlog:", self.confidence_backlog)
                print("DETECTOR:  backlog size:", len(self.confidence_backlog))
                if len(self.confidence_backlog) > 5:
                    self.confidence_backlog.pop(0)
                if len(self.confidence_backlog) == 5:
                    average_of_confidences = sum(self.confidence_backlog) / len(self.confidence_backlog)

                    distance, angle, vertical = stereo_vision(spread=0.8636,
                                                              center_right=best_right.Center,
                                                              center_left=best_left.Center,
                                                              fov=62.2,
                                                              image_width=left_image.width,
                                                              image_height=left_image.height)
                    if not math.isnan(distance):
                        self.logfile.write("Official distance:" + str(distance) + "\n")
                        self.logfile.write("Official horizontal angle:" + str(angle) + "\n")
                        self.logfile.write("Official vertical angle:" + str(vertical) + "\n")
                        persons_cords_local = self.GPS.get_persons_GPS(distance, angle, location_snapsave,
                                                                       vertical, self.camera_offset)
                        # persons_cords_local = self.GPS.get_persons_GPS(distance, angle, location_snapsave)
                        timestamp = time.time()
                        self.logfile.write("Official coordinates:" + str(persons_cords_local) + "\n\n")
                        print("DETECTOR:  persons coordinates are:", self.persons_cords)

            self.allow_read = False
            self.persons_cords = persons_cords_local
            self.detection_timestamp = timestamp
            self.best_left = best_left
            self.best_right = best_right
            self.people_on_the_left = people_left
            self.people_on_the_right = people_right
            self.average = average_of_confidences
            self.allow_read = True

    def get_best_detection(self, detections: list):
        best_detection = None
        people_detected_list = []

        if len(detections) > 0:
            for detection in detections:
                if self.net.GetClassDesc(detection.ClassID) == 'person':
                    people_detected_list.append(detection)
                    if best_detection is None or detection.Confidence > best_detection.Confidence:
                        best_detection = detection
        else:
            return None, None
        return best_detection, people_detected_list

    def is_thread_alive(self):
        return self.thread.is_alive()

    def kill(self):
        self.isRunning = False
        self.thread.join()
        self.logfile.write("Thread killed, closing the file.")
        self.logfile.close()
        self.left_camera.kill()
        self.right_camera.kill()


def kill(core):
    core.kill()


if __name__ == '__main__':
    GPS = MavLink("/dev/ttyACM0", 57600)
    detector = Detector("0", "1", "test_left", "test_left", "./Recordings", 0, GPS, flip=True)
    atexit.register(kill, detector)
