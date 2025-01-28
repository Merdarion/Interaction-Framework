import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import asyncio
import numpy as np
import os
import matplotlib.pyplot as plt

from g3pylib import connect_to_glasses

class EyeTrackerVideoPublisher(Node):
    """
    A ROS2 node that captures video from an RTSP stream and publishes it as ROS Image messages.

    Attributes:
        image_publisher_ (Publisher): The ROS2 publisher for Image messages.
        bridge (CvBridge): The bridge to convert between ROS Image messages and OpenCV images.
    """
    def __init__(self):
        """
        Initialize the EyeTrackerVideoPublisher node, create the publisher, and set up the CvBridge.
        """
        super().__init__("EyeTrackerVideoPublisher")
        self.add_msg_to_info_logger("initializing node")
        self.scene_camera_publisher = self.create_publisher(Image, 'eyetracker_camera', 10)
        self.gaze_publisher = self.create_publisher(String, "gaze_coordinates", 10)
        self.bridge = CvBridge()
        
    def publish_scene_camera(self, cv2_image):
        """
        Publish a given OpenCV image as a ROS Image message.

        Args:
            cv2_image (numpy.ndarray): The OpenCV image to be published.

        Raises:
            CvBridgeError: If conversion from OpenCV image to ROS Image message fails.
        """
        try:
             msg = self.bridge.cv2_to_imgmsg(cv2_image, "bgr8")
             self.scene_camera_publisher.publish(msg)
        except CvBridgeError as e:
             self.add_msg_to_info_logger(e)
        #self.add_msg_to_info_logger("sending_image")

    def publish_gaze_data(self, gaze):
        """
        Publish a given OpenCV image as a ROS Image message.

        Args:
            cv2_image (numpy.ndarray): The OpenCV image to be published.

        Raises:
            CvBridgeError: If conversion from OpenCV image to ROS Image message fails.
        """
        try:
             msg = String()
             msg.data = gaze
             self.gaze_publisher.publish(msg)
        except CvBridgeError as e:
             self.add_msg_to_info_logger(e)
        #self.add_msg_to_info_logger("sending_image")
        

    async def connect_to_tobii_glasses(self):
        async with connect_to_glasses.with_hostname(
            # Add g3-address to the environment variable "G3_HOSTNAME" -> see documentation of the g3pylib Python API
            os.environ["G3_HOSTNAME"], using_zeroconf=True
        ) as g3:
            async with g3.stream_rtsp(scene_camera=True, gaze=True) as streams:
                async with streams.gaze.decode() as gaze_stream, streams.scene_camera.decode() as scene_stream:
                    while True:
                        frame, frame_timestamp = await scene_stream.get()
                        gaze, gaze_timestamp = await gaze_stream.get()
                        while gaze_timestamp is None or frame_timestamp is None:
                            if frame_timestamp is None:
                                frame, frame_timestamp = await scene_stream.get()
                            if gaze_timestamp is None:
                                gaze, gaze_timestamp = await gaze_stream.get()
                        while gaze_timestamp < frame_timestamp:
                            gaze, gaze_timestamp = await gaze_stream.get()
                            while gaze_timestamp is None:
                                gaze, gaze_timestamp = await gaze_stream.get()

                        frame = frame.to_ndarray(format="bgr24")

                        # If given gaze data
                        if "gaze2d" in gaze:
                            gaze2d = gaze["gaze2d"]

                            # Convert rational (x,y) to pixel location (x,y)
                            h, w = frame.shape[:2]
                            fix = str([int(gaze2d[0] * w), int(gaze2d[1] * h)])
                            self.publish_gaze_data(fix)
                                            
                        self.publish_scene_camera(frame)

                        

    def add_msg_to_info_logger(self, msg):
        """
        Log a message at the info level.

        Args:
            msg (str): The message to log.
        """
        self.get_logger().info(msg)

def main(args=None):
    """
    The main entry point for the script. Initializes the ROS2 node, captures video from an RTSP stream,
    and publishes the video frames as ROS Image messages until 'q' is pressed.

    Args:
        args (list, optional): Command-line arguments passed to the script.
    """

    rclpy.init(args=args)

    rtsp_publisher = EyeTrackerVideoPublisher()

    rtsp_publisher.add_msg_to_info_logger("initializing camera")

    asyncio.run(rtsp_publisher.connect_to_tobii_glasses())

    rclpy.spin(rtsp_publisher)

    

if __name__ == "__main__":
    main()
