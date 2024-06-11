import cv2
import numpy as np
from inference import get_model
from inference import InferencePipeline
from inference.core.interfaces.camera.entities import VideoFrame
import supervision as sv
        
annotator = sv.BoxAnnotator()

# model = get_model("10m-images/8")

def my_custom_sink(predictions: dict, video_frame: VideoFrame):
    labels = [p["class"] for p in predictions["predictions"]]
    detections = sv.Detections.from_inference(predictions)
    # annotate the frame using our supervision annotator, the video_frame, the predictions (as supervision Detections), and the prediction labels
    image = annotator.annotate(
        scene=video_frame.image.copy(), detections=detections, labels=labels
    )
    # display the annotated image
    cv2.imshow("Predictions", image)


# Replace the model_id with the model_id of your choice and the video ref with the drone live feed
pipeline = InferencePipeline.init(
    model_id="yolov8x-1280",
    video_reference="https://storage.googleapis.com/com-roboflow-marketing/inference/people-walking.mp4",
    on_prediction=my_custom_sink,
)

pipeline.start()
pipeline.join()