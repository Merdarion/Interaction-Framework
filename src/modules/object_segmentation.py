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
#from object_detection_tracking import ObjectDetectionAndTracking



class ObjectSegmentation():
    def __init__(self):
        self.predictor = SAM2ImagePredictor.from_pretrained("facebook/sam2-hiera-large")
        self.coordinates = []
        self.colors = []



    def segment(self, rgb_image, track, depth_image=None):
        self.coordinates = []
        self.colors = []
        with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
            self.predictor.set_image(rgb_image)
            boxes = track.xyxy
            results = []
            label = []

            masks, _, _ = self.predictor.predict(
                                    point_coords=None,
                                    point_labels=None,
                                    box=boxes,
                                    multimask_output=False
            )

            mask = masks.squeeze()


            for i, mask in enumerate(masks):
                object_coordinates, _ = self.calculate_coordinates(mask, depth_image)
                self.coordinates.append(object_coordinates)
                label.append(track["class_name"][0])

            results = np.concatenate(masks, axis=0)
            
            return label, self.coordinates, results
        
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
            cz = 10
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
                num_masks = squeezed_masks.shape[0] if squeezed_masks.ndim > 0 else 0
                
                if num_masks == 1080:
                    segmentation = [(idx, squeezed_masks, boxes[idx]) for idx in range(len(boxes))]
                else:
                    segmentation = [(idx, squeezed_masks[idx], boxes[idx]) for idx in range(len(boxes))]


                for i, mask, box in segmentation:
                    # print(f"Processing mask index: {i}")
                    # print(f"Bounding box: {box}")
                    # print(mask.shape)  # This should be (1080, 1920) for each mask

                    # print(f"Gaze coordinates: {gaze}")  # Debugging gaze coordinates

                    # Ensure gaze is valid
                    if 0 <= gaze[0] < 1920 and 0 <= gaze[1] < 1080:
                        mask_value = mask[gaze[1], gaze[0]]  # Access the correct pixel

                        # Check the conditions based on the mask value
                        if mask_value == 1.0:
                            object_color = detect_color(image, box)
                            if object_color == "yellow":
                                fixated_object_label = "box with " + object_color + "cubes"
                            else:
                                fixated_object_label = track[i]["class_name"][0]

                        # Check if gaze is within bounding box
                        if box[0] <= gaze[0] <= box[2] and box[1] <= gaze[1] <= box[3]:
                            object_color = detect_color(image, box)
                            if object_color == "yellow":
                                fixated_object_label = "box with " + object_color + "cubes"
                            else:
                                fixated_object_label = track[i]["class_name"][0]


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

    t = np.array([[10], [-600], [-750]])

    camera_coordinates = k_inv @ pixel_coordinates
    #print(camera_coordinates)

    depth_camera_coordinates = w * camera_coordinates
    #print(depth_camera_coordinates)

    world_coordinates = R @ (depth_camera_coordinates - t)

    x, y, z = world_coordinates.flatten()
    x, y, z = int(round(x)), int(round(y)), int(round(z))

    return x, y, z


def detect_color(image, box):
    # Extract the region of the bounding box
    crop = image[int(box[1]):int(box[3]), int(box[0]):int(box[2])]

    # Convert ROI to HSV
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

    # Define color ranges in HSV
    orange_lower = np.array([5, 150, 150])
    orange_upper = np.array([25, 255, 255])
    yellow_lower = np.array([20, 100, 100])
    yellow_upper = np.array([35, 255, 255])

    red_lower1 = np.array([0, 60, 20])
    red_upper1 = np.array([20, 100, 50])
    red_lower2 = np.array([160, 60, 20])
    red_upper2 = np.array([180, 100, 50])

    # Green
    green_lower = np.array([35, 60, 30])
    green_upper = np.array([85, 80, 60])

    # # Cyan (Aqua)
    cyan_lower = np.array([85, 100, 100])
    cyan_upper = np.array([95, 255, 255])

    # Blue
    blue_lower = np.array([95, 100, 100])
    blue_upper = np.array([130, 255, 255])

    # Purple
    purple_lower = np.array([130, 50, 50])
    purple_upper = np.array([160, 255, 255])

    # Pink (Magenta)
    pink_lower = np.array([160, 100, 100])
    pink_upper = np.array([170, 255, 255])

    # Brown
    brown_lower = np.array([10, 100, 20])
    brown_upper = np.array([20, 255, 200])

    # Masks for each color
    mask_lower_red = cv2.inRange(hsv, red_lower1, red_upper1)
    mask_upper_red = cv2.inRange(hsv, red_lower2, red_upper2)
    mask_orange = cv2.inRange(hsv, orange_lower, orange_upper)
    mask_yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)
    mask_green = cv2.inRange(hsv, green_lower, green_upper)
    mask_cyan = cv2.inRange(hsv, cyan_lower, cyan_upper)
    mask_blue = cv2.inRange(hsv, blue_lower, blue_upper)
    mask_purple = cv2.inRange(hsv, purple_lower, purple_upper)
    mask_pink = cv2.inRange(hsv, pink_lower, pink_upper)
    mask_brown = cv2.inRange(hsv, brown_lower, brown_upper)


    # Calculate the percentage of each color
    orange_percent = (mask_orange > 0).mean() * 100
    yellow_percent = (mask_yellow > 0).mean() * 100
    lower_red_percent = (mask_lower_red > 0).mean() * 100
    upper_red_percent = (mask_upper_red > 0).mean() * 100
    purple_percent = (mask_purple > 0).mean() * 100
    green_percent = (mask_green > 0).mean() * 100
    cyan_percent = (mask_cyan > 0).mean() * 100
    blue_percent = (mask_blue > 0).mean() * 100
    pink_percent = (mask_pink > 0).mean() * 100
    brown_percent = (mask_brown > 0).mean() * 100

    if yellow_percent > 0:
        color = "yellow"
        return color
    
    elif green_percent > 0:
        color = "green"
        return color
    
    elif (lower_red_percent or upper_red_percent) > 0:
        color = "red"
        return color

    elif orange_percent > 0:
        color = "orange"
        return color


    # # Determine the dominant color
    # colors = {
    #     'orange': orange_percent,
    #     'yellow': yellow_percent,
    #     'red': lower_red_percent + upper_red_percent,
    #     'purple': purple_percent,
    #     'green': green_percent,
    #     'cyan': cyan_percent,
    #     'blue': blue_percent,
    #     'pink': pink_percent,
    #     'brown': brown_percent,
    # }
    # dominant_color = max(colors, key=colors.get)

    # return dominant_color, colors


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
#     # Ensure mask is binary (0s and 1s)
#     mask = (mask > 0).astype(np.uint8) * 255

#     # Create a color mask (for visualization purposes)
#     color_mask = np.zeros_like(image)
#     color_mask[mask == 255] = [255, 0, 0]  # Red color for the mask

#     # Overlay the mask on the original image
#     overlay = cv2.addWeighted(image, 0.7, color_mask, 0.3, 0)

#     # Plotting
#     plt.figure(figsize=(12, 6))

#     # Plot original image with mask overlay
#     plt.subplot(1, 2, 1)
#     plt.imshow(cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB))
#     plt.title(f'{title} - Image with Mask')
#     plt.axis('off')

#     # Plot the mask alone
#     plt.subplot(1, 2, 2)
#     plt.imshow(mask, cmap='gray')
#     plt.title(f'{title} - Mask Only')
#     plt.axis('off')

#     plt.show()

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

def main():
    # import pykinect_azure as pykinect

    # yolo_model = ObjectDetectionAndTracking()
    # segmentation = ObjectSegmentation()

    # pykinect.initialize_libraries()

	# # Modify camera configuration
    # device_config = pykinect.default_configuration
    # device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1080P
    # device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED
    # #print(device_config)

	# # Start device
    # device = pykinect.start_device(config=device_config)

    # while True:
    #     capture = device.update()

    #     rgb_ret, rgb_frame = capture.get_color_image()
    #     depth_ret, depth_image = capture.get_transformed_depth_image()

    #     if rgb_ret:
    #         print("Frame captured.")

    #         detect = yolo_model.detect(rgb_frame, verbose=False)
    #         track = yolo_model.track(rgb_frame)
    #         rgb_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2RGB)

    #         if depth_ret:
    #             coordinates, masks = segmentation.segment(rgb_frame, detect, depth_image)
    #             print(detect)
    #             print(coordinates)

    image = cv2.imread("/home/aass/ros2_bident_ws/src/modules/test.png")
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    yolo_model = ObjectDetectionAndTracking()
    detect = yolo_model.detect(image, verbose=True)
    detect = yolo_model.track(image)


    objects = []

    # for object in detect:
    #     coords, _, _, _, _, name = object
    #     color, colors = detect_color(image, coords)

    #     objects.append(color + " " + name["class_name"])
    #     print(colors, name["class_name"])

    plot_detections(image, detect)




if __name__ == "__main__":
    main()
