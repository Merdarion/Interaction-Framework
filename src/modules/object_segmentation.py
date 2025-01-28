import matplotlib.pyplot as plt
import numpy as np
import torch
import supervision as sv
from sam2.sam2_image_predictor import SAM2ImagePredictor
import cv2
import os

import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.getcwd(), 'src')))
from src.modules.object_detection_tracking import ObjectDetectionAndTracking



class ObjectSegmentation():
    def __init__(self):
        self.predictor = SAM2ImagePredictor.from_pretrained("facebook/sam2-hiera-large")
        self.coordinates = []



    def segment(self, rgb_image, track, depth_image):
        self.coordinates = []
        with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
            self.predictor.set_image(rgb_image)
            boxes = track.xyxy
            results = []

            masks, _, _ = self.predictor.predict(
                                    point_coords=None,
                                    point_labels=None,
                                    box=boxes,
                                    multimask_output=False
            )

            for i, mask in enumerate(masks):
                object_coordinates, _ = self.calculate_coordinates(mask, depth_image)
                self.coordinates.append(object_coordinates)

            results = np.concatenate(masks, axis=0)
            
            return self.coordinates, results
        
    def calculate_coordinates(self, mask, depth_image):

        coordinates = []
        mask = mask.squeeze()

        # Calculate moments of the mask
        M = cv2.moments(mask)

        # Calculate the centroid coordinates (relative to the original image's coordinate system)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            cx, cy = 0, 0
        #print(cx, cy)

        # Find the average depth value of an object in the depth_image -> taking only the centroid would sometimes cause to depth_value = 0 (if there is no depth_data at this coordinate)
        depth_values = depth_image[(mask == 1) & (depth_image > 0)]
        avg_depth_value = depth_values.mean()

        try:
            cz = int(avg_depth_value.round())
        except ValueError:
            pass
        #print(cz)
        #cz = 1

        # Getting the world coordinates
        try:
            x, y, z = convert_2d_to_3d(cx, cy, cz)
            coordinates = [x, y, z]

        except Exception:
            pass


        return coordinates, mask

    def segment_fixation(self, image, track, gaze_data):
        with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
            self.predictor.set_image(image)
            boxes = track.xyxy
            fixated_object_label = ""
            gaze = [int(coord) for coord in gaze_data]

            try:
                masks, _, _ = self.predictor.predict(
                                        point_coords=None,
                                        point_labels=None,
                                        box=boxes,
                                        multimask_output=False
                )

                squeezed_masks = masks.squeeze()

                # Check for dimensionality of the masks -> the it has to be a squeezed mask with dimensionalit
                num_masks = squeezed_masks.shape[0] if squeezed_masks.ndim > 0 else 0
                
                if num_masks == 1080:
                    segmentation = [(idx, squeezed_masks, boxes[idx]) for idx in range(len(boxes))]
                else:
                    segmentation = [(idx, squeezed_masks[idx], boxes[idx]) for idx in range(len(boxes))]


                for i, mask, box in segmentation:

                    # Ensure gaze coordinates are valid -> sometimes they are 
                    if 0 <= gaze[0] < 1920 and 0 <= gaze[1] < 1080:
                        mask_value = mask[gaze[1], gaze[0]]
                        print(f"Mask value at gaze: {mask_value}")
                    else:
                        # Gaze coordinates are out of bounds
                        pass

                    # Check if the mask value on the gaze coordinate is 1 (if the gaze lies within the mask)
                    if mask_value == 1.0:
                        fixated_object_label = track[i]["class_name"][0]
                        print(f"Fixated object label from mask: {fixated_object_label}")

                    # Check if gaze coordinate is within a bounding box -> additional check; gaze data is not always 100% precise
                    elif box[0] <= gaze[0] <= box[2] and box[1] <= gaze[1] <= box[3]:
                        fixated_object_label = track[i]["class_name"][0]
                        print(f"Fixated object label from bounding box: {fixated_object_label}")


            except (AssertionError, IndexError):
                pass
            
            return fixated_object_label

def convert_2d_to_3d(u, v, w, f=972.577):

    # These are the intrinsics of the RGB camera according to the output of the Kinect-ROS-Driver
    K = np.array([[f, 0, 1026.3],
                    [0, f, 774.433],
                    [0, 0, 1]])
    k_inv = np.linalg.inv(K)
    pixel_coordinates = np.array([[u], [v], [1]])

    # The NAO is sitting behind the camera in a fixed position but his coordinate system
    # is rotated, x is the approach, y the open, z the normal vector
    R = np.array([[0, 0, 1],
                [-1, 0, 0],
                [0, -1, 0]])

    # Adjust these values here for the placement of the NAO robot behind the Kinect
    t = np.array([[10], [-600], [-750]])

    camera_coordinates = k_inv @ pixel_coordinates
    #print(camera_coordinates)

    depth_camera_coordinates = w * camera_coordinates
    #print(depth_camera_coordinates)

    world_coordinates = R @ (depth_camera_coordinates - t)

    x, y, z = world_coordinates.flatten()
    x, y, z = int(round(x)), int(round(y)), int(round(z))

    return x, y, z



# The following functions are only for debugging purposes
# def plot_mask(image, mask, title='Mask Plot'):
#     """
#     Plots the mask on top of the original image and shows the mask alone.
    
#     :param image: The original RGB image (H, W, C) where H is height, W is width, C is color channels.
#     :param mask: The binary mask (H, W) where object pixels are 1 and background pixels are 0.
#     :param title: The title for the plot.
#     """

#     if mask.ndim == 3 and mask.shape[0] == 1:
#         mask = mask.squeeze(0)  # Remove extra dimension
#     Ensure mask is binary (0s and 1s)
#     mask = (mask > 0).astype(np.uint8) * 255

#     Create a color mask (for visualization purposes)
#     color_mask = np.zeros_like(image)
#     color_mask[mask == 255] = [255, 0, 0]  # Red color for the mask

#     Overlay the mask on the original image
#     overlay = cv2.addWeighted(image, 0.7, color_mask, 0.3, 0)

#     Plotting
#     plt.figure(figsize=(12, 6))

#     Plot original image with mask overlay
#     plt.subplot(1, 2, 1)
#     plt.imshow(cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB))
#     plt.title(f'{title} - Image with Mask')
#     plt.axis('off')

#     Plot the mask alone
#     plt.subplot(1, 2, 2)
#     plt.imshow(mask, cmap='gray')
#     plt.title(f'{title} - Mask Only')
#     plt.axis('off')

#     plt.show()

# def plot_detections(image, detections):
#     plt.figure(figsize=(10, 10))
#     plt.imshow(image)

#     for i, box in enumerate(detections.xyxy):
#         x1, y1, x2, y2 = map(int, box[:4])
#         label = detections.data['class_name'][i]
#         confidence = detections.confidence[i]
        
#         # Draw the bounding box
#         plt.gca().add_patch(plt.Rectangle((x1, y1), x2-x1, y2-y1, edgecolor='red', facecolor='none', linewidth=2))
        
#         # Add label and confidence score
#         plt.text(x1, y1, f'{label} {confidence:.2f}', color='white', fontsize=12, bbox=dict(facecolor='red', alpha=0.5))

#     plt.axis('off')
#     plt.imshow()

def main():
    import pykinect_azure as pykinect

    yolo_model = ObjectDetectionAndTracking()
    segmentation = ObjectSegmentation()

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
        depth_ret, depth_image = capture.get_transformed_depth_image()

        if rgb_ret:
            print("Frame captured.")

            detect = yolo_model.detect(rgb_frame, verbose=False)
            track = yolo_model.track(rgb_frame)
            rgb_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2RGB)

            if depth_ret:
                coordinates, masks = segmentation.segment(rgb_frame, detect, depth_image)
                print(detect)
                print(coordinates)


if __name__ == "__main__":
    main()
