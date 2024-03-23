import math
import time
import Fuse
import jetson_inference
from Triangulation import stereo_vision


class DetectorThreadRunner:
    def __init__(self, l_camera, r_camera, pitch, spread, fov, GPS_node, log_file):
        self.l_camera = l_camera
        self.r_camera = r_camera
        self.pitch = pitch
        self.spread = spread
        self.fov = fov
        self.GPS = GPS_node
        self.d_net = jetson_inference.detectNet()
        self.person_loc = None
        self.detection_time = None
        self.log_f = log_file
        self.detect_fuse = Fuse.Fuse()

    def swap_cameras(self):
        self.l_camera, self.r_camera = self.r_camera, self.l_camera

    def thread_loop(self):
        r_image = None
        l_image = None
        image_location = None
        l_detections = None
        r_detections = None
        best_detection_l = None
        best_detection_r = None
        person_location_internal = None
        time_stamp_internal = None

        while not self.l_camera.allow_read:
            pass
        l_image = self.l_camera.img

        while not self.r_camera.allow_read:
            pass
        r_image = self.r_camera.img

        while not self.GPS.allow_read:
            pass
        image_location = self.GPS.loc

        l_detections = self.d_net.Detect(l_image, overlay="box,labels,conf")
        r_detections = self.d_net.Detect(r_image, overlay="box,labels,conf")

        best_detection_l, all_detections_l = self.get_best_detection(l_detections)
        best_detection_r, all_detections_r = self.get_best_detection(r_detections)

        if best_detection_l is None or best_detection_r is None:
            self.detect_fuse.punish()
        else:
            self.detect_fuse.reward()
            if self.detect_fuse:
                # High confidence triangulation this will cause class attributes to update
                self.person_loc, self.detection_time = self.get_persons_loc(l_image, best_detection_l,
                                                                            best_detection_r, image_location,
                                                                            print_mode="h")
            else:
                # low confidence triangulation, just prints to a file
                self.get_persons_loc(l_image, best_detection_l, best_detection_r, image_location, print_mode="l")

        if person_location_internal is not None:
            self.person_loc = person_location_internal
            self.detection_time = time_stamp_internal

    def kill(self):
        self.l_camera.kill()
        self.r_camera.kill()

    def get_best_detection(self, detections: list):
        best_detection = None
        people_detected_list = []

        if len(detections) > 0:
            for detection in detections:
                if self.d_net.GetClassDesc(detection.ClassID) == 'person':
                    people_detected_list.append(detection)
                    if best_detection is None or detection.Confidence > best_detection.Confidence:
                        best_detection = detection
        else:
            return None, None
        return best_detection, people_detected_list

    def get_persons_loc(self, image, best_l, best_r, location, print_mode=None):
        time_stamp = None
        prefix = "Low confidence"
        distance, angle, vertical = stereo_vision(spread=0.8636,
                                                  center_right=best_r.Center,
                                                  center_left=best_l.Center,
                                                  fov=62.2,
                                                  image_width=image.width,
                                                  image_height=image.height)
        if not math.isnan(distance):
            persons_cords_local = self.GPS.get_persons_GPS(distance, angle, location,
                                                           vertical, self.pitch)
            time_stamp = time.time()
            if print_mode is not None:
                if print_mode == "h":
                    prefix = ">>> High confidence"
                self.log_f.write(prefix + " distance:" + str(distance) + "\n")
                self.log_f.write(prefix + " horizontal angle:" + str(angle) + "\n")
                self.log_f.write(prefix + " vertical angle:" + str(vertical) + "\n")
                self.log_f.write(prefix + " coordinates:" + str(persons_cords_local) + "\n\n")
            return persons_cords_local, time_stamp
        return None, None
