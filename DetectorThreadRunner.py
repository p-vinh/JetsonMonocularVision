#!/usr/bin/env python3

"""
   The purpose of this class it to separate variables used inside a detection and geolocation thread and other threads
   that take data from that thread. Mixing up variables between two threads might cause thread conflicts, unless
   operations done on those variables are thread safe. DO NOT BYPASS Detector.py AND READ FOM THIS CLASSES INSTANCE
   ATTRIBUTES. USE GETTERS METHODS IN Detector.py INSTANCE.
"""

import math
import time
import Fuse
from ultralytics import YOLO
from Triangulation import stereo_vision
from sahi.predict import predict
import os

# Left camera will be used as a main camera. Right camera will be used as a reference camera.
class DetectorThreadRunner:
    def __init__(self, l_camera, r_camera, pitch, spread, fov, t_fuse, GPS_node, log_file):
        self.l_camera = l_camera
        self.r_camera = r_camera
        self.pitch = pitch
        self.spread = spread
        self.fov = fov
        self.GPS = GPS_node
        self.d_net = YOLO('model/weed_model.pt') # Trained YOLOv8 model
        self.person_loc = None
        self.detection_time = None
        self.log_f = log_file
        self.detect_fuse = Fuse.Fuse(t_fuse)

    def swap_cameras(self):
        self.l_camera, self.r_camera = self.r_camera, self.l_camera

    def thread_loop(self):
        r_image = None
        l_image = None
        image_location = None
        l_detections = []
        r_detections = None
        best_detection_l = None
        best_detection_r = None
        person_location_internal = None
        time_stamp_internal = None

        # Get images from both cameras. A list of images is returned from the camera thread. The first image in the top right corner of the original image.
        while not self.l_camera.allow_read:
            pass
        l_image = self.l_camera.img 

        while not self.r_camera.allow_read:
            pass
        r_image = self.r_camera.img
        
        # Get detections from the YOLOv8 model through SAHI Helper
        l_detections = predict(
            model_type='yolov8',
            model_path='model/weed_model.pt',
            model_config_path='args_for_sahi.yaml',
            model_device='cuda:0',
            model_confidence_threshold=0.6,
            postprocess_class_agnostic=True,
            source=l_image,
            slice_height=480,
            slice_width=640,
            overlap_height_ratio=0.2,
            overlap_width_ratio=0.2,
            visual_bbox_thickness=1,
            visual_text_size=0.5,
            visual_text_thickness=1,
            export_pickle=False
        )
        
        # Get GPS location from Mavlink module. Scene GPS is read at about the same time its safe to assume that the
        # location stores is tah location at which those pictures were taken.
        while not self.GPS.allow_read:
            pass
        image_location = self.GPS.loc
        

        # Go through list of objects detected on the left and find a box with the highest confidence value.
        # "all_detections_l" variable is not used anywhere it's just a leftover from debugging.
        best_detection_l, all_detections_l = self.get_best_detection(l_detections)
        # Do the same for right image
        # best_detection_r, all_detections_r = self.get_best_detection(r_detections)

        """
            For triangulation to work with multiple targets, triangulation code has to know which detection in left 
            image correlates to which detection in the right image. In this case an assumption is made: the detection
            with the highest confidence in the left image corresponds to the right image's highest confidence detection.
            So, best detection in the left image is the same target as the right image best detection. Then 
            triangulation will be performed on those two detections. 
        """

        # chaneg this to use the pre-processed images and not the fuse (threshold) class
        # if best_detection_l is None or best_detection_r is None:
        #     # If only one of the cameras got a detection or neither, the that means it might be a false positive or,
        #     # target is too far away. In ether case triangulation should be delayed. So, the fuse is punished (fined).
        #     self.detect_fuse.punish()
        # else:
        #     # If both images got a detection, the reward the fuse, indicating that detections is likely is not a false
        #     # positive and target is close.
        #     self.detect_fuse.reward()
        #     if self.detect_fuse:
        #         # If fuse fused, that there were multiple high confidence detection in a short period of time. So,
        #         # a triangulation is performed and the results of this triangulation will be sent to other threads.
        #         self.person_loc, self.detection_time = self.get_persons_loc(l_image, best_detection_l,
        #                                                                     best_detection_r, image_location,
        #                                                                     self.spread, self.fov,
        #                                                                     print_mode="h")
        #     else:
        #         # low confidence triangulation, just prints to a file. Results of this triangulation will not be sent to
        #         # other threads.
        #         self.get_persons_loc(l_image, best_detection_l, best_detection_r, self.spread, self.fov,
        #                              image_location, print_mode="l")

        if person_location_internal is not None:
            self.person_loc = person_location_internal
            self.detection_time = time_stamp_internal

    def kill(self):
        self.l_camera.kill()
        self.r_camera.kill()

    #TODO: Determine what is returned by the YOLOv8 model 
    def get_best_detection(self, detections):
        best_detection = None
        weed_detected_list = []

        if detections is not None and len(detections) > 0:
            for i in range(len(detections.class_id)):
                # Check to see if the detection is a 'weed'
                if detections.class_id[i] == 1 and detections.data['class_name'][i] == 'weed':
                    weed_detected_list.append(detections[i])
                    if best_detection is None or detections.confidence[i] > best_detection.confidence:
                        best_detection = detections[i]
        else:
            return None, None
        return best_detection, weed_detected_list

    def get_weed_loc(self, image, best_l, best_r, spread, fov, location, print_mode=None):
        time_stamp = None
        prefix = "Low confidence"
        weed_cords_local = None
        # Triangulation function will perform calculations to get direct distance, horizontal angle, and a vertical
        # angle to the target. Returns None if calculation fails.
        distance, angle, vertical = stereo_vision(spread=spread,
                                                  center_right=best_r[:2],  # Assuming center coordinates are at index 0 and 1
                                                  center_left=best_l[:2],  # Assuming center coordinates are at index 0 and 1
                                                  fov=fov,
                                                  image_width=image.shape[1],
                                                  image_height=image.shape[0])
        if not math.isnan(distance):
            # If triangulation calculation did not fail, then use "get_weed_GPS()" method to combine, GPS, heading,
            # horizontal and vertical angles, camera pitch angle, and distance to predict targets GPS location.
            # weed_cords_local = self.GPS.get_weed_GPS(distance, angle, location,
            #                                                vertical, self.pitch)
            weed_cords_local = (0, 0)
            # Save the timestamp of when this triangulation happened.
            time_stamp = time.time()
            # Print to log file.
            if print_mode is not None:
                if print_mode == "h":
                    prefix = ">>> High confidence"
                self.log_f.write(prefix + " distance:" + str(distance) + "\n")
                self.log_f.write(prefix + " horizontal angle:" + str(angle) + "\n")
                self.log_f.write(prefix + " vertical angle:" + str(vertical) + "\n")
                self.log_f.write(prefix + " coordinates:" + str(weed_cords_local) + "\n\n")
            # Return weed GPS and time when this triangulation happened.
            return weed_cords_local, time_stamp
        # Return None when triangulation fails.
        return None, None
