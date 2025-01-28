import sys
import os
sys.path.append(os.path.abspath(os.getcwd()))

from queue import Queue
from cv_bridge import CvBridge, CvBridgeError
from getkey import getkey

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
import supervision as sv
import matplotlib.pyplot as plt

from time import sleep

from src.modules.scene_description import SceneDescription
from src.modules.object_detection_tracking import ObjectDetectionAndTracking
from src.modules.object_segmentation import ObjectSegmentation
from src.modules.action_module import ActionGeneration

from geometry_msgs.msg import PoseStamped

class MergedInputPublisher(Node):
    """
    A ROS 2 node that publishes audio transcriptions as messages.

    This class defines a ROS 2 node that publishes text messages representing audio transcriptions. It initializes
    the ROS node, sets up a publisher, and includes methods to publish messages and log information.

    Methods:
        __init__(): Initializes the AudioPublisher node, sets up the publisher, and logs the start message.
        publish(text): Publishes a text message to the 'audio' topic.
        add_msg_to_info_logger(msg): Logs an informational message using the node's logger.
    """
    def __init__(self):
        """
        Initializes the AudioPublisher node.

        Sets up the ROS 2 node with the name 'AudioPublisher', creates a publisher for the 'audio' topic, and logs
        an informational start message.
        """
        super().__init__("MergedInputPublisher")
        self.publisher = self.create_publisher(String, 'merged_input', 10)
        timer_period = 0.25
        self.create_timer(timer_period, self.publish)
        self.bridge = CvBridge()

        global activity
        activity = ActionGeneration
        self.gaze_data = String()
        self.eyetracker_image = String()
        self.rgb_image = np.ndarray
        self.depth_image = np.ndarray
        self.final_message = dict()
        self.tracked = []
        self.scene_description = ""
        self.segmented = []
        self.coordinates = []
        self.initialized = False
        self.track_objects = False
        self.block_transmission = False
        self.eye_tracking = False

        self.audio_queue = Queue()

        self.object_detector = ObjectDetectionAndTracking()
        self.fixated_object_detector = ObjectDetectionAndTracking()
        self.scene_describer = SceneDescription()
        self.object_segmentator = ObjectSegmentation()

        # Collecting all the different inputs
        self.audio_subscription = self.create_subscription(String, 'audio', self.get_audio, 10)
        self.eyetracker_camera_subscription = self.create_subscription(Image, 'eyetracker_camera', self.eyetracker_camera_callback, 10)
        self.gaze_subscription = self.create_subscription(String, 'gaze_coordinates', self.gaze_data_callback, 10)
        self.rgb_subscription = self.create_subscription(Image, 'rgb_image', self.rgb_camera_callback, 10)
        self.depth_subscription = self.create_subscription(Image, 'depth_image', self.depth_camera_callback, 10)
        self.keyboard_subscription = self.create_subscription(String, 'key_input', self.keyboard_callback, 10)

        self.rgb_subscription
        self.depth_subscription
        self.audio_subscription
        self.keyboard_subscription
        self.gaze_subscription
        self.eyetracker_camera_subscription

        # For the qualisys integration of the experiment later
        # Add Variables for the Qualisys Callbacks
        # self.azure_kinect_topic = "/qualisys/Azure_Kinect/pose"
        # self.eyetracker_topic = "/qualisys/Tobii_Glasses3/pose"

        # self.citi_subscription = self.create_subscription(PoseStamped, self.azure_kinect_topic, self.kinect_callback, 10)
        # self.eyetracker_position_subscription = self.create_subscription(PoseStamped, self.eyetracker_topic, self.eyetracker_callback, 10)

        # Flags for handling_commands
        # self.new_decision = False

        # self.eyetracker_position = [0., 0., 0.]
        # self.eyetracker_rotation = [0., 0., 0.]
        # self.kinect_position = [0., 0., 0.]
        # self.kinect_rotation = [0., 0., 0.]

        # def kinect_callback(self, kinect_msg):
        #     """Subscribes to qualisys/Citi_1 topic and returns the psoiton in naos_robot_frame
        #         NOTE Only for Demo purposes (Position of cititruck is interesting, but in world coordinates)
        #     """

        #     try:
        #         self.kinect_position = (kinect_msg.pose.position.x, kinect_msg.pose.position.y, kinect_msg.pose.position.z)
        #         self.kinect_quaternion = (kinect_msg.pose.rotation.w, kinect_msg.pose.rotation.x, kinect_msg.pose.rotation.y, kinect_msg.pose.rotation.z)
        #     except Exception as e:
        #         print("ERROR in in Qualisys Citi Callback",e)

        # def eyetracker_callback(self, eyetracker_callback):
        #     """Subscribes to given qualisys/Helmet_Number topics and returns the positon in naos_robot_frame"""

        #     self.eyetracker_position = [
        #         eyetracker_callback.pose.position.x, 
        #         eyetracker_callback.pose.position.y, 
        #         eyetracker_callback.pose.position.z
        #     ]
        #     self.eyetracker_rotation = [eyetracker_callback.pose.rotation.w,
        #                                 eyetracker_callback.pose.rotation.x,
        #                                 eyetracker_callback.pose.rotation.y,
        #                                 eyetracker_callback.pose.rotation.z
        #                                 ]        




    def get_audio(self, msg):
        """
        Receives messages from the 'audio' topic and adds them to the message queue.

        Args:
            msg (std_msgs.msg.String): The message received from the 'audio' topic.
        """
        if msg.data != "":
            self.audio_queue.put(msg.data)
            #self.get_logger().info(f"Got a message: \"{msg.data}\" Response is being generated...")
    


    def rgb_camera_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
     
    def depth_camera_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")

    def eyetracker_camera_callback(self, msg):
        self.eyetracker_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    
    def gaze_data_callback(self, msg):
        self.gaze_data = list(msg.data.strip('[]').replace(" ", "").split(','))

    def keyboard_callback(self, msg):
        if msg.data == "t":
            self.track_objects = not self.track_objects
        
        if msg.data == "b":
            self.block_transmission = not self.block_transmission
    
    def merger(self):
        self.final_message = None
        self.segmented = None

        if self.initialized == False and type(self.rgb_image) == np.ndarray: # Scene description gets initialized 
            try:
                self.scene_description = self.scene_describer.describe(self.rgb_image)
                self.detected = self.object_detector.detect(self.rgb_image, verbose=True)
                self.tracked = self.object_detector.track(self.rgb_image)
                self.initialized = True if self.detected is not None else False

            except TypeError:
                self.scene_description = None


        if self.track_objects:
            self.detected = self.object_detector.detect(self.rgb_image, verbose=True)
            self.tracked = self.object_detector.track(self.rgb_image)

        # Collecting the input - only if the user said something!
        if not self.audio_queue.empty():
            input_dictionary = dict()

            # Clears the transcription if something wrong got transcribed (like background noise)
            if self.block_transmission:
                self.audio_queue.get(block=False)
                self.add_msg_to_info_logger("Transmission blocked")
            else:
                input_dictionary["transcription"] = self.audio_queue.get().lower()

                if input_dictionary["transcription"] in ("you", "thank you", "thank you!", "thank you.", "you thank you", "you  thank you"):
                    self.add_msg_to_info_logger("Transcription was likely an hallucination. Did not send.")
                else:
                    self.coordinates, _ = self.object_segmentator.segment(rgb_image=self.rgb_image, track=self.tracked, depth_image=self.depth_image)            
                    
                    if not isinstance(self.gaze_data, String) and not isinstance(self.eyetracker_image, String):
                        detect_fixated_object = self.object_detector.detect(self.eyetracker_image, verbose=True)
                        fixated_object_label = self.object_segmentator.segment_fixation(image=self.eyetracker_image, track=detect_fixated_object, gaze_data=self.gaze_data)
                    else:
                        fixated_object_label = ""

                    if self.detected:
                        input_dictionary["object_labels"] = list(self.detected["class_name"])
                    else:
                        input_dictionary["object_labels"] = []

                    input_dictionary["fixated_object_label"] = fixated_object_label

                    if self.coordinates:
                        input_dictionary["object_coordinates"] = self.coordinates
                    else:
                        input_dictionary["object_coordinates"] = []


                    self.final_message = str(input_dictionary)



    def publish(self):
        self.merger()
        msg = String()

        if self.final_message is not None:
            msg.data = self.final_message
            self.add_msg_to_info_logger(msg.data)
            self.publisher.publish(msg)
        
    def add_msg_to_info_logger(self, msg):
        """
        Log a message at the info level.

        Args:
            msg (str): The message to log.
        """
        self.get_logger().info(msg)


def main():
    """
    Initializes the ROS 2 Python client library, creates a ResponsePublisher node, logs a message, 
    and spins the node to keep it alive.

    Args:
        args (list, optional): Command line arguments. Defaults to None.
    """
    rclpy.init()

    merged_input = MergedInputPublisher()
    merged_input.get_logger().info("Merging input now...") 

    rclpy.spin(merged_input)

    #Destroy the node explicitly
    #(optional - otherwise it will be done automatically
    #when the garbage collector destroys the node object)
    merged_input.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()