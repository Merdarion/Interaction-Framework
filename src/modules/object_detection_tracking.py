import cv2
import matplotlib.pyplot as plt
import sys
from ultralytics import YOLOWorld
import os
import numpy as np
import supervision as sv

from time import sleep

import warnings
warnings.filterwarnings('ignore')

from PIL import Image

# def plot_detections(image, detections):
#     """
#     Draws bounding boxes and labels on the image for each detection.
    
#     Args:
#         image (numpy.ndarray): The image where detections will be drawn.
#         detections (Detections): Object containing bounding box coordinates, labels, and confidence scores.
#     """
#     for i, box in enumerate(detections.xyxy):
#         x1, y1, x2, y2 = map(int, box[:4])
#         label = detections.data['class_name'][i]
#         confidence = detections.confidence[i]
        
#         # Draw the bounding box
#         cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
        
#         # Add label and confidence score
#         text = f'{label} {confidence:.2f}'
#         cv2.putText(image, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

#     plt.axis("off")
#     plt.imshow()


def plot_detections(image, detections):
    plt.figure(figsize=(10, 10))
    plt.imshow(image)

    for i, box in enumerate(detections.xyxy):
        x1, y1, x2, y2 = map(int, box[:4])
        label = detections.data['class_name'][i]
        confidence = detections.confidence[i]
        
        # Draw the bounding box
        plt.gca().add_patch(plt.Rectangle((x1, y1), x2-x1, y2-y1, edgecolor='red', facecolor='none', linewidth=2))
        
        # Add label and confidence score
        plt.text(x1, y1, f'{label} {confidence:.2f}', color='white', fontsize=12, bbox=dict(facecolor='red', alpha=0.5))

    plt.axis('off')
    plt.show()
    

class ObjectDetectionAndTracking():
    def __init__(self):
        global model
        model = YOLOWorld(os.path.join(os.getcwd(), './src/modules/models/thesis_experiment.pt')
        # Add model hyperparameters here with self.model.hyperparam1 etc.
        )
        self.model = model
        self.results = []

        self.model.conf = 0.5
        self.model.iou = 0.7

    def detect(self, input, verbose):
        try:
            if not input == []:
                self.results = self.model.predict(input, verbose=verbose, conf=self.model.conf, iou=self.model.iou)[0] # stream=True is not necessary, frames are processed individually instead of having one large video file -> see ultralytics docs
                detections = sv.Detections.from_ultralytics(self.results)
                #print("Detections: ", detections)

                return detections
            else:
                print("Waiting for frame...")

        except KeyboardInterrupt:
            sys.exit(0)

    def track(self, input):
        try:
            if not input == []:
                results = self.model.track(input, persist=True, tracker="bytetrack.yaml", conf=self.model.conf, iou=self.model.iou)[0]
                tracked = sv.Detections.from_ultralytics(results)

                return tracked
            else:
                print("Waiting for frame...")
        
        except KeyboardInterrupt:
            sys.exit(0)



class ObjectTracking():
    def __init__(self):
        global model
        self.tracker = sv.ByteTrack()

    def track(self, input):
        tracked_input = self.tracker.update_with_detections(input)
        return tracked_input

def main():

    object = ObjectDetectionAndTracking()
    tracking = ObjectTracking()

    import pykinect_azure as pykinect

    pykinect.initialize_libraries()

	# Modify camera configuration
    device_config = pykinect.default_configuration
    device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1080P
    device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED
    #print(device_config)

	# Start device
    device = pykinect.start_device(config=device_config)

    while True:
        capture = device.update()

        rgb_ret, rgb_frame = capture.get_color_image()

        if rgb_ret:
            print("Frame captured.")
            rgb_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2RGB)

            detections = object.detect(rgb_frame, verbose=False)
            if detections:
                print(detections)
                plot_detections(rgb_frame, detections)

                print("Objects detected.")
                
                tracked_objects = object.track(rgb_frame)
                print(tracked_objects)

                plot_detections(rgb_frame, tracked_objects)

                print("Objects tracked.")
            else:
                print("No detections found.")
        else:
            print("No RGB frame captured.")



if __name__ == "__main__":
    main()
