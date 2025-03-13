"""
   The purpose of this class it to separate variables used inside a detection and geolocation thread and other threads
   that take data from that thread. Mixing up variables between two threads might cause thread conflicts, unless
   operations done on those variables are thread safe. DO NOT BYPASS Detector.py AND READ FOM THIS CLASSES INSTANCE
   ATTRIBUTES. USE GETTERS METHODS IN Detector.py INSTANCE.
"""

#!/usr/bin/python3.8
import math
import time
#from ultralytics import YOLO
from Triangulation import stereo_vision, monocular_vision, calculate_gsd
import os
#from sahi.predict import get_sliced_prediction
#from sahi import AutoDetectionModel
import numpy as np
import cv2
import tempfile
from PIL import Image
import jetson_utils
from inference_sdk import InferenceHTTPClient


# Left camera will be used as a main camera. For Monocular vision
class DetectorThreadRunner:
    def __init__(self, l_camera, pitch, spread, sensor_width, fov, GPS_node, log_file):
        API_KEY = os.getenv("API_KEY")
        self.l_camera = l_camera
        self.pitch = pitch
        self.spread = spread
        self.fov = fov
        self.sensor_width = sensor_width
        self.GPS = GPS_node
        # self.d_net = YOLO("model/best.pt")  # Trained YOLOv8 model
 #       print("CREATING DETECTION MODEL FROM PRE TRAINED")
 #       self.detection_model = AutoDetectionModel.from_pretrained(
#	        model_type="ultralytics",
#	        model_path="model/best.pt",
#	        confidence_threshold=0.4,
#	        device="cuda:0",
#	    	)
        self.client = InferenceHTTPClient(api_url="https://detect.roboflow.com", api_key=API_KEY)
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

        try:
                l_image_np = jetson_utils.cudaToNumpy(l_image)
                l_image_pil = Image.fromarray(l_image_np)

                #l_detections = get_sliced_prediction(
                #    image=l_image_pil,
                #    detection_model=self.detection_model,
                #    postprocess_class_agnostic=True,
                #    overlap_height_ratio=0.2,
                #    overlap_width_ratio=0.2,
                #    auto_slice_resolution=True,
                #)
                l_detections = self.client.run_workflow(
		    workspace_name="strawberries-fx9j1",
		    workflow_id="small-human-detection",
		    images={
			"image": l_image_pil
		    },
		    use_cache=True
		)
                detections = l_detections[0]["predictions"]["predictions"]
                #print(detections)
                
                
        except Exception as e:
            print(e)
            return
        # Get GPS location from Mavlink module. Scene GPS is read at about the same time its safe to assume that the
        # location stores is tah location at which those pictures were taken.
        while not self.GPS.allow_read:
            pass
        image_location = self.GPS.loc

        # Go through list of objects detected on the left and find a box with the highest confidence value.
        # "all_detections_l" variable is not used anywhere it's just a leftover from debugging.
        if (detections is not None):
             #best_detection_l, all_detections_l = self.get_best_detection(l_detections.object_prediction_list)
             best_detection_l, all_detections_l = self.get_best_detection(detections)
        else:
             print("No weed predictions detected.")
             return

        if best_detection_l is None:
            print("Cannot find any weed detection in the image")
            return
        
        print("Best detection: ", best_detection_l)
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

        if detections is not None and len(detections) > 0:
            for i in range(len(detections)):
                # Check to see if the detection is a 'weed'
                if (detections[i]["class_id"] == 1 and detections[i]["class"] == "weed"):
                    weed_detected_list.append(detections[i])
                    if (best_detection is None or detections[i]["confidence"] > best_detection["confidence"]):
                        best_detection = detections[i]

                # TEST ONLY W/ Human detection Model:
                if (detections[i]["class_id"] == 0 and detections[i]["class"] == "person"):
                    weed_detected_list.append(detections[i])
                    if (best_detection is None or detections[i]["confidence"] > best_detection["confidence"]):
                       best_detection = detections[i]
 
         # OLD CODE MIGHT NEED LATER    
         #if detections is not None and len(detections) > 0:
         #   for i in range(len(detections)):
         #       # Check to see if the detection is a 'weed'
         #       if (
         #           detections[i].category.id == 1
         #           and detections[i].category.name == "weed"
         #       ):
         #           weed_detected_list.append(detections[i])
         #           if (
         #               best_detection is None
         #               or detections[i].score.value > best_detection.score.value
         #           ):
         #               best_detection = detections[i]
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
        gsd = calculate_gsd(fov, sensor_width, image.shape[1], location["alt"])
        
        # Calculate the center of the bounding box.
        #x_center = (best_l.bbox.minx + best_l.bbox.maxx) / 2
        #y_center = (best_l.bbox.miny + best_l.bbox.maxy) / 2
        
        x_center = best_l["x"]
        y_center = best_l["y"]
        
        lat, lon = monocular_vision(
            location["lat"],
            location["lon"],
            location["alt"],
            location["hdg"],
            gsd,
            image.shape[1],
            image.shape[0],
            x_center,
            y_center,
        )
        
        print("Lat: ", lat, " Lon: ", lon)

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
    
    def kill(self):
        self.thread.join()
        self.l_camera.kill()
        return
