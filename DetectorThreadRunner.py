"""
   The purpose of this class it to separate variables used inside a detection and geolocation thread and other threads
   that take data from that thread. Mixing up variables between two threads might cause thread conflicts, unless
   operations done on those variables are thread safe. DO NOT BYPASS Detector.py AND READ FOM THIS CLASSES INSTANCE
   ATTRIBUTES. USE GETTERS METHODS IN Detector.py INSTANCE.
"""

#!/usr/bin/python3.8
import math
import time
import Fuse
from ultralytics import YOLO
from Triangulation import stereo_vision, monocular_vision, calculate_gsd
from sahi.slicing import slice_image
import os
import numpy as np
from PIL import Image, ImageDraw, ImageFont

SLICE_SIZE = 800
SLICE_OVERLAP = 0.2

# Left camera will be used as a main camera. For Monocular vision
class DetectorThreadRunner:
    def __init__(self, l_camera, pitch, spread, sensor_width, fov, GPS_node, log_file):
        self.l_camera = l_camera
        self.pitch = pitch
        self.spread = spread
        self.fov = fov
        self.sensor_width = sensor_width
        self.GPS = GPS_node
        self.d_net = YOLO("model/weed_model.pt")  # Trained YOLOv8 model
        self.weed_loc = None
        self.detection_time = None
        self.log_f = log_file

    def thread_loop(self):
        l_image = None
        image_location = None
        l_detections = []
        best_detection_l = None
        time_stamp_internal = None

        # Get images from both cameras. A list of images is returned from the camera thread. The first image in the top right corner of the original image.
        while not self.l_camera.allow_read:
            pass
        l_image = self.l_camera.img

        model = YOLO("model/weed_model.pt")
        # Get detections from the YOLOv8 model through SAHI Helper
        # l_detections = predict(
        #     model_type='yolov8',
        #     model_path='model/weed_model.pt',
        #     model_config_path='args_for_sahi.yaml',
        #     model_device='cuda:0',
        #     model_confidence_threshold=0.6,
        #     postprocess_class_agnostic=True,
        #     source=l_image,
        #     slice_height=480,
        #     slice_width=640,
        #     overlap_height_ratio=0.2,
        #     overlap_width_ratio=0.2,
        #     visual_bbox_thickness=1,
        #     visual_text_size=0.5,
        #     visual_text_thickness=1,
        # )

        # l_detections = sv.Detections.from_ultralytics(model(l_image)[0])
        
        background_img = Image.open(l_image)

        slice_image_result = slice_image(
            image=l_image,
            slice_height=SLICE_SIZE,
            slice_width=SLICE_SIZE,
            overlap_height_ratio=SLICE_OVERLAP,
            overlap_width_ratio=SLICE_OVERLAP,
        )

        print(f"Number of slices: {len(slice_image_result)}")

        # Extract bounding box coordinates+identified cls and store in detections
        for index, sliced_image in enumerate(slice_image_result.sliced_image_list):
            img = sliced_image.image
            PIL_img = Image.fromarray(img)

            results = model.predict(
                source=PIL_img,
                line_width=2,
                verbose=False,
                max_det=5000,
                device="cpu",
                conf=0.5,
                iou=0.7,
            )

            boxes = results[0].boxes
            x, y = sliced_image.starting_pixel
            for box in boxes:
                cls_index = int(box.cls[0])
                x1 = box.xyxy[0][0] + x
                y1 = box.xyxy[0][1] + y
                x2 = box.xyxy[0][2] + x
                y2 = box.xyxy[0][3] + y
                l_detections.append(
                    {"cls_index": cls_index, "x1": x1, "y1": y1, "x2": x2, "y2": y2}
                )
        inference_time = time.time()
        print(f"Number of ddetections: {len(l_detections)}")


        # Get GPS location from Mavlink module. Scene GPS is read at about the same time its safe to assume that the
        # location stores is tah location at which those pictures were taken.
        while not self.GPS.allow_read:
            pass
        image_location = self.GPS.loc

        # Go through list of objects detected on the left and find a box with the highest confidence value.
        # "all_detections_l" variable is not used anywhere it's just a leftover from debugging.
        best_detection_l, all_detections_l = self.get_best_detection(l_detections)

        if best_detection_l is None:
            raise Exception("Cannot find any weed detection in the image")

        self.weed_loc, self.detection_time = self.get_weed_loc(
            l_image,
            best_detection_l,
            self.sensor_width,
            self.fov,
            image_location,
            print_mode="h",
        )

        if self.weed_loc is not None:
            self.weed_loc = self.weed_loc
            self.detection_time = time_stamp_internal

        if self.log_f is not None:
            self.log_f.write("Detection time: " + str(self.detection_time) + "\n")
            self.log_f.write("Person location: " + str(self.weed_loc) + "\n\n")

    def kill(self):
        self.l_camera.kill()

    # TODO: Determine what is returned by the YOLOv8 model
    def get_best_detection(self, detections):
        best_detection = None
        weed_detected_list = []
        print(detections)
        if detections is not None and len(detections) > 0:
            for i in range(len(detections.class_id)):
                # Check to see if the detection is a 'weed'
                if (
                    detections.class_id[i] == 1
                    and detections.data["class_name"][i] == "weed"
                ):
                    weed_detected_list.append(detections[i])
                    if (
                        best_detection is None
                        or detections.confidence[i] > best_detection.confidence
                    ):
                        best_detection = detections[i]
        else:
            return None, None
        return best_detection, weed_detected_list

    def get_weed_loc(self, image, best_l, sensor_width, fov, location, print_mode=None):
        time_stamp = None
        prefix = "Low confidence"
        weed_cords_local = None
        # Triangulation function will perform calculations to get direct distance, horizontal angle, and a vertical
        # angle to the target. Returns None if calculation fails. !REQUIRES RIGHT IMAGE FOR STEREO VISION!
        # distance, angle, vertical = stereo_vision(spread=spread,
        #                                           center_right=best_r[:2],  # Assuming center coordinates are at index 0 and 1
        #                                           center_left=best_l[:2],  # Assuming center coordinates are at index 0 and 1
        #                                           fov=fov,
        #                                           image_width=image.shape[1],
        #                                           image_height=image.shape[0])

        # Calculate the distance, and coordinates using Monocular vision.
        # Note Altitude is set to 10meters. Change Mavlink to get the actual altitude.
        ALTITUDE = 10
        # Test: 5472x3648
        gsd = calculate_gsd(fov, sensor_width, image.shape[1], ALTITUDE)
        lat, lon = monocular_vision(
            location[0],
            location[1],
            ALTITUDE,
            gsd,
            image.shape[1],
            image.shape[0],
            best_l[0],
            best_l[1],
        )

        if lat is not None and lon is not None:
            # If triangulation calculation did not fail, then use "get_weed_GPS()" method to combine, GPS, heading,
            # horizontal and vertical angles, camera pitch angle, and distance to predict targets GPS location.
            # weed_cords_local = self.GPS.get_weed_GPS(distance, angle, location,
            #                                                vertical, self.pitch)
            weed_cords_local = {"lat": lat, "lon": lon}
            time_stamp = time.time()
            # Print to log file.
            if print_mode is not None:
                if print_mode == "h":
                    prefix = ">>> High confidence"
                self.log_f.write(
                    prefix
                    + "Monocular Vision: Latitude: "
                    + str(lat)
                    + " Longitude: "
                    + str(lon)
                    + "\n\n"
                )
                self.log_f.write(prefix + "Time: " + str(time_stamp) + "\n")
                self.log_f.write(
                    "------------------------------------------------------\n"
                )
            # Return weed GPS and time when this triangulation happened.
            return weed_cords_local, time_stamp
        # Return None when triangulation fails.
        return None, None
