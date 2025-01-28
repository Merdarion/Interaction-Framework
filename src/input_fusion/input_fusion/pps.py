import sys
import os
sys.path.append(os.path.abspath(os.getcwd()))

from queue import Queue
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
import numpy as np
from time import sleep

from datetime import datetime

from src.modules.object_detection_tracking import ObjectDetectionAndTracking
from src.modules.object_segmentation import ObjectSegmentation
from src.modules.scene_description import SceneDescription
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
        self.bridge = CvBridge()

        self.gaze_data = String()
        self.eyetracker_image = String()
        self.rgb_image = np.ndarray
        self.depth_image = np.ndarray
        self.final_message = None
        self.tracked = []
        self.segmented = []
        self.coordinates = []
        self.distance_to_person = 99999
        self.distance_human_to_tools = 99999

        self.tool_moved = False
        self.tool_placed = False
        self.more_than_one_tool_moved = False
        self.initialized = False
        self.person_next_to_nao = False


        self.step = 0
        self.tool_counter = 0
        self.tools_counter = 0
        self.tool_moved_counter = 0
        self.distance_counter = 0

        # Check the last fixations
        self.tool1_counter = 0
        self.tool2_counter = 0
        self.tool3_counter = 0

        self.audio_queue = Queue()


        self.object_detector = ObjectDetectionAndTracking()
        self.fixated_object_detector = ObjectDetectionAndTracking()
        self.object_segmentator = ObjectSegmentation()
        self.scene_description = SceneDescription()

        # Collecting all the different inputs
        self.eyetracker_camera_subscription = self.create_subscription(Image, 'eyetracker_camera', self.eyetracker_camera_callback, 10)
        self.gaze_subscription = self.create_subscription(String, 'gaze_coordinates', self.gaze_data_callback, 10)
        self.rgb_subscription = self.create_subscription(Image, 'rgb_image', self.rgb_camera_callback, 10)
        self.depth_subscription = self.create_subscription(Image, 'depth_image', self.depth_camera_callback, 10)
        self.audio_subscription = self.create_subscription(String, 'audio', self.get_audio, 10)

        self.transcribe_publisher = self.create_publisher(Bool, 'prevent_loopback', 10)


        self.transcribe = False
        self.prevent_loopback(self.transcribe)


        self.rgb_subscription
        self.depth_subscription
        self.gaze_subscription
        self.eyetracker_camera_subscription
        self.audio_subscription

        # Add Variables for the Qualisys Callbacks
        self.tool1_topic = "/qualisys/Tool_1/pose"
        self.tool2_topic = "/qualisys/Tool_2/pose"
        self.tool3_topic = "/qualisys/Tool_3/pose"

        self.box1_topic = "/qualisys/Box_1/pose"
        self.box2_topic = "/qualisys/Box_2/pose"
        self.box3_topic = "/qualisys/Box_3/pose"
        self.box4_topic = "/qualisys/Box_4/pose"

        self.citi_topic = "/qualisys/Citi_2/pose"
        self.eyetracker_topic = "/qualisys/Tobii_Glasses3/pose"

        self.tool1_subscription = self.create_subscription(PoseStamped, self.tool1_topic, self.tool1_callback, 10)
        self.tool2_subscription = self.create_subscription(PoseStamped, self.tool2_topic, self.tool2_callback, 10)
        self.tool3_subscription = self.create_subscription(PoseStamped, self.tool3_topic, self.tool3_callback, 10)

        self.box1_subscription = self.create_subscription(PoseStamped, self.box1_topic, self.box1_callback, 10)
        self.box2_subscription = self.create_subscription(PoseStamped, self.box2_topic, self.box2_callback, 10)
        self.box3_subscription = self.create_subscription(PoseStamped, self.box3_topic, self.box3_callback, 10)
        self.box4_subscription = self.create_subscription(PoseStamped, self.box4_topic, self.box4_callback, 10)

        self.citi_subscription = self.create_subscription(PoseStamped, self.citi_topic, self.citi_callback, 10)
        self.eyetracker_position_subscription = self.create_subscription(PoseStamped, self.eyetracker_topic, self.eyetracker_position_callback, 10)

        # Flags for handling_commands
        self.tool1_is_moved = False
        self.tool2_is_moved = False
        self.tool3_is_moved = False

        self.tool1_position = [0., 0., 0.]
        self.tool2_position = [0., 0., 0.]
        self.tool3_position = [0., 0., 0.]

        self.tool1_init_position = [0, 0, 0]
        self.tool2_init_position = [0, 0, 0]
        self.tool3_init_position = [0, 0, 0]

        self.box1_position = [0., 0., 0.]
        self.box2_position = [0., 0., 0.]
        self.box3_position = [0., 0., 0.]
        self.box4_position = [0., 0., 0.]

        self.toolbox_position = [0., 0., 0.]

        self.citi_position = [0., 0., 0.]
        self.eyetracker_position = [0., 0., 0.]

        # Inferred varibales from Qualisys

        self.detected = {"class_name": []}
        self.detect_fixated_object = {"class_name": []}


    def rgb_camera_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
     
    def depth_camera_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")

    def eyetracker_camera_callback(self, msg):
        self.eyetracker_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    
    def gaze_data_callback(self, msg):
        self.gaze_data = list(msg.data.strip('[]').replace(" ", "").split(','))

    def prevent_loopback(self, transcription_flag):
        """Attempt to prevent the Whisper module to transcribe again what NAO said"""

        msg = Bool()
        msg.data = transcription_flag
        self.transcribe_publisher.publish(msg)

    def get_audio(self, msg):
        """
        Receives messages from the 'audio' topic and adds them to the message queue.

        Args:
            msg (std_msgs.msg.String): The message received from the 'audio' topic.
        """
        if msg.data != "":
            self.audio_queue.put(msg.data)
            self.create_user_message()
            #self.get_logger().info(f"Got a message: \"{msg.data}\" Response is being generated...")
    
    def action_decision(self):
        """Evalutes detection data and decides, if command needs to be sent
            Returns after decision for command has been made
            This function is the CORE FUNCTION for Automatical mode
        """

        # Since we need Object Detection and Segmentation here, let's rather check the fixations in the action_decision rather than in the 
        # Callback functions -> less lag

        # if "human" in self.detected["class_name"] and np.count_nonzero(self.detected["class_name"] == "human") < 2:
        #     objects, coordinates, _ = self.object_segmentator.segment(self.rgb_image, self.detected, self.depth_image)
        #     self.human_current_position = next((coord for obj, coord in zip(objects, coordinates) if obj == 'human'), self.transformations())
        #     self.distance_to_person = np.linalg.norm(np.array(self.eyetracker_position) - np.array(self.citi_position)) * 1000
        # else:
        self.human_current_position = self.transformations()
        self.distance_to_person = np.linalg.norm(np.array(self.eyetracker_position) - np.array(self.citi_position)) * 1000

        boxes_centre = (np.array(self.box1_position) + np.array(self.box2_position) + np.array(self.box3_position) + np.array(self.box4_position)) / 4

        tools_centre = (np.array(self.tool1_position) + np.array(self.tool2_position) + np.array(self.tool3_position)) / 3

        self.distance_human_to_tools = np.linalg.norm(np.array(self.eyetracker_position) - np.array(tools_centre)) * 1000
        self.distance_human_to_boxes = np.linalg.norm(np.array(self.eyetracker_position) - np.array(boxes_centre)) * 1000
        #########################

        if self.eyetracker_position != [0.0, 0.0, 0.0] and self.distance_to_person < 2000:
            self.distance_counter += 1
            if self.distance_counter > 5:
                self.person_next_to_nao = True

        # self.person_next_to_nao = False

        # Step by step plan
        if self.step == 0:

            if self.person_next_to_nao and self.distance_human_to_tools > 0.0:
                print("Initial step triggered")
                print("Distance to NAO:", self.distance_to_person, "\n")

                sleep(5)

                self.transcribe = False
                self.prevent_loopback(self.transcribe)
                actor.thinking()

                actor.talker("Hello colleague. Thank you for bringing me the can. Now please help me with another task.")

                self.step += 1

                return

        elif self.step == 1:

            actor.onCallLook(*self.human_current_position)

            print("First step triggered", "\n")

            sleep(2)

            actor.talker("Go in front of the table with the green plant on it and wait there.")

            self.tool1_init_position = self.tool1_position
            self.tool2_init_position = self.tool2_position
            self.tool3_init_position = self.tool3_position


            point_to_tools = self.transformations(to="tools")

            actor.onCallPoint(*point_to_tools)
            actor.onCallLook(*point_to_tools)


            sleep(2)

            self.audio_queue.get(block=False) if not self.audio_queue.empty() else None
            self.transcribe = True
            self.prevent_loopback(self.transcribe)

            actor.listening()

            self.step += 1

        elif self.step == 2:

            if 0.0 < self.distance_human_to_tools < 1500:
                print("Step 2 triggered")
                print("Distance to tools:", self.distance_human_to_tools, "\n")

                self.transcribe = False
                self.prevent_loopback(self.transcribe)

                actor.thinking()

                actor.onCallLook(*self.human_current_position)
                actor.talker("Take one of the tools in front of you and show it to me.")

                sleep(2)

                self.audio_queue.get(block=False) if not self.audio_queue.empty() else None
                self.transcribe = True
                self.prevent_loopback(self.transcribe)
                
                actor.listening()

                self.step += 1


        elif self.step == 3:

            self.tool_move_check()

            if self.more_than_one_tool_moved and self.tools_counter < 2:
                self.transcribe = False
                self.prevent_loopback(self.transcribe)

                print("No new step triggered.")
                print("More than one tool moved.", "\n")

                actor.thinking()

                actor.onCallLook(*self.human_current_position)
                actor.talker("It seems that you took more than one tool. Put them back on the table and take only one of it.")

                sleep(2)

                self.audio_queue.get(block=False) if not self.audio_queue.empty() else None
                self.transcribe = True
                self.prevent_loopback(self.transcribe)

                actor.listening()

            elif self.tool_moved:
                print("Step 3 triggered")
                print("Tool moved", "\n")

                self.transcribe = False
                self.prevent_loopback(self.transcribe)

                actor.thinking()

                actor.onCallLook(*self.human_current_position)
                actor.talker("Go with the tool to the four blue boxes on the table and wait there.")

                point_to_boxes = self.transformations(to="boxes")
                actor.onCallPoint(*point_to_boxes)
                actor.onCallLook(*point_to_boxes)


                sleep(2)

                self.audio_queue.get(block=False) if not self.audio_queue.empty() else None
                self.transcribe = True
                self.prevent_loopback(self.transcribe)

                actor.listening()
                self.step += 1               

        elif self.step == 4:

            self.tool_move_check()

            if self.more_than_one_tool_moved and self.tools_counter < 2:
                self.transcribe = False
                self.prevent_loopback(self.transcribe)

                print("No new step triggered.")
                print("More than one tool moved.", "\n")

                actor.thinking()

                actor.onCallLook(*self.human_current_position)
                actor.talker("It seems that you took more than one tool. Put them back on the table and take only one of it.")

                sleep(2)

                self.audio_queue.get(block=False) if not self.audio_queue.empty() else None
                self.transcribe = True
                self.prevent_loopback(self.transcribe)

                actor.listening()

            elif 0.0 < self.distance_human_to_boxes < 1500:
                self.transcribe = False
                self.prevent_loopback(self.transcribe)

                print("Step 4 triggered")
                print("Distance to Boxes:", self.distance_human_to_boxes, "\n")

                actor.thinking()

                actor.onCallLook(*self.human_current_position)
                actor.talker("Show me the tool again. Afterwards, put it in the box that I will show you now.")

                point_to_left_box_back = self.transformations(to="box")
                point_to_left_box_back[1] += 0.5
                actor.onCallPoint(*point_to_left_box_back)
                actor.onCallLook(*point_to_left_box_back)

                self.audio_queue.get(block=False) if not self.audio_queue.empty() else None
                self.transcribe = True
                self.prevent_loopback(self.transcribe)

                actor.listening()

                self.step += 1

        elif self.step == 5:

            # Check if tool is in box
            distance_tool1_to_box4 = np.linalg.norm(np.array(self.tool1_position) - np.array(self.box4_position)) * 1000
            distance_tool2_to_box4 = np.linalg.norm(np.array(self.tool2_position) - np.array(self.box4_position)) * 1000
            distance_tool3_to_box4 = np.linalg.norm(np.array(self.tool3_position) - np.array(self.box4_position)) * 1000

            distance_tool1_to_box2 = np.linalg.norm(np.array(self.tool1_position) - np.array(self.box2_position)) * 1000
            distance_tool2_to_box2 = np.linalg.norm(np.array(self.tool2_position) - np.array(self.box2_position)) * 1000
            distance_tool3_to_box2 = np.linalg.norm(np.array(self.tool3_position) - np.array(self.box2_position)) * 1000

            distances = [distance_tool1_to_box2, distance_tool1_to_box4, distance_tool2_to_box2, distance_tool2_to_box4, distance_tool3_to_box2, distance_tool3_to_box4]

            if any(0 < distance <= 150 for distance in distances):
                self.tool_counter += 1
                if self.tool_counter >= 5:
                    self.tool_placed = True

            if self.tool_placed:
                self.transcribe = False
                self.prevent_loopback(self.transcribe)
                actor.thinking()

                print("Final step triggered", "\n")

                actor.onCallLook(*self.human_current_position)
                actor.talker("Thank you for your help. Now go back to the yellow area behind me.")

                self.step += 1

                #actor.onCallPoint(*[0, 1, 0])      


    def tool1_callback(self, tool_msg):
        self.tool1_position = [tool_msg.pose.position.x, tool_msg.pose.position.y, tool_msg.pose.position.z]
        self.tool1_counter += 1

    def tool2_callback(self, tool_msg):
        self.tool2_position = [tool_msg.pose.position.x, tool_msg.pose.position.y, tool_msg.pose.position.z]
        self.tool2_counter += 1


    def tool3_callback(self, tool_msg):
        self.tool3_position = [tool_msg.pose.position.x, tool_msg.pose.position.y, tool_msg.pose.position.z]
        self.tool3_counter += 1


    def box1_callback(self, box_msg):
        self.box1_position = [box_msg.pose.position.x, box_msg.pose.position.y, box_msg.pose.position.z]

    def box2_callback(self, box_msg):
        self.box2_position = [box_msg.pose.position.x, box_msg.pose.position.y, box_msg.pose.position.z]

    def box3_callback(self, box_msg):
        self.box3_position = [box_msg.pose.position.x, box_msg.pose.position.y, box_msg.pose.position.z]

    def box4_callback(self, box_msg):
        self.box4_position = [box_msg.pose.position.x, box_msg.pose.position.y, box_msg.pose.position.z]


    def citi_callback(self, citi_msg):
        """Subscribes to qualisys/Citi_2 topic and returns the psoiton in naos_robot_frame
            NOTE Only for Demo purposes (Position of cititruck is interesting, but in world coordinates)
        """
        self.citi_position = [citi_msg.pose.position.x, citi_msg.pose.position.y, citi_msg.pose.position.z]#

    def eyetracker_position_callback(self, eyetracker_callback):
        """Subscribes to given qualisys/Helmet_Number topics and returns the positon in naos_robot_frame"""

        self.eyetracker_position = [
            eyetracker_callback.pose.position.x, 
            eyetracker_callback.pose.position.y, 
            eyetracker_callback.pose.position.z
        ]
        self.action_decision()

    def tool_move_check(self):
        self.distance_tool1_to_init_tool1 = np.linalg.norm(np.array(self.tool1_position) - np.array(self.tool1_init_position)) * 1000
        self.distance_tool2_to_init_tool2 = np.linalg.norm(np.array(self.tool2_position) - np.array(self.tool2_init_position)) * 1000
        self.distance_tool3_to_init_tool3 = np.linalg.norm(np.array(self.tool3_position) - np.array(self.tool3_init_position)) * 1000

        # This tool is less precise than the others, don't know why...
        if 500 <= self.distance_tool1_to_init_tool1:
            self.tool1_is_moved = True
        else:
            self.tool1_is_moved = False
            
        if 300 <= self.distance_tool2_to_init_tool2:
            self.tool2_is_moved = True
        else:
            self.tool2_is_moved = False

        if 300 <= self.distance_tool3_to_init_tool3:
            self.tool3_is_moved = True
        else:
            self.tool3_is_moved = False


        if self.tool1_is_moved ^ self.tool2_is_moved ^ self.tool3_is_moved:
            self.tool_moved_counter += 1
            if self.tool_moved_counter >= 5:
                self.tool_moved = True
                self.more_than_one_tool_moved = False
                self.tools_counter = 0
        elif self.tool1_is_moved or self.tool2_is_moved or self.tool3_is_moved:
            self.more_than_one_tool_moved = True
            self.tools_counter += 1
            self.tool1_counter = self.tool2_counter = self.tool3_counter = 0
        #########################

    def create_user_message(self):
        self.segmented = None

        if self.initialized == False and type(self.rgb_image) == np.ndarray: # Scene description gets initialized 
            try:
                self.scene_description = self.scene_description.describe(self.rgb_image)
                self.initialized = True if self.scene_description is not None else False

            except TypeError:
                self.scene_description = None

        try:
            self.detected = self.object_detector.detect(self.rgb_image, verbose=False)
        except (AttributeError, TypeError):
            pass


        # Collecting the input - only if the user said something!
        if not self.audio_queue.empty():
            input_dictionary = dict()

            # Clears the transcription if something wrong got transcribed (like background noise)
            input_dictionary["transcription"] = self.audio_queue.get().lower()

            if input_dictionary["transcription"] in ("you", "thank you", "thank you!", "thank you.", "you thank you", "you  thank you"):
                self.add_msg_to_info_logger("Transcription was likely an hallucination. Did not send.")
            else:
                # try:
                #     self.coordinates, _ = self.object_segmentator.segment(rgb_image=self.rgb_image, track=self.tracked, depth_image=self.depth_image)            
                # except AssertionError:
                #     pass

                if not isinstance(self.gaze_data, String) and not isinstance(self.eyetracker_image, String):
                    detect_fixated_object = self.object_detector.detect(self.eyetracker_image, verbose=True)
                    fixated_object_label = self.object_segmentator.segment_fixation(image=self.eyetracker_image, track=detect_fixated_object, gaze_data=self.gaze_data)
                else:
                    fixated_object_label = ""

                input_dictionary["object_coordinates"] = []
                input_dictionary["fixated_object_label"] = fixated_object_label


                if self.detected is not None:
                    if "human" in list(self.detected["class_name"]):
                        input_dictionary["object_labels"] = ["human"]
                        input_dictionary["object_coordinates"].append([self.human_current_position[0], self.human_current_position[1], self.human_current_position[2]])
                    else:
                        input_dictionary["object_labels"] = []

                else:
                    input_dictionary["object_labels"] = []

                if self.scene_description is not None:
                    input_dictionary["scene_description"] = self.scene_description
                else:
                    input_dictionary["scene_description"] = ""

                    input_dictionary["object_coordinates"] = []

                    if self.tool_moved:
                        input_dictionary["tool_taken"] = True
                    else:
                        input_dictionary["tool_taken"] = False

                input_dictionary["object_labels"].append("tools")
                point_to_tools = self.transformations(to="tools")
                input_dictionary["object_coordinates"].append([point_to_tools[0], point_to_tools[1], point_to_tools[2]])

                input_dictionary["object_labels"].append("correct box")
                point_to_box = self.transformations(to="box")
                input_dictionary["object_coordinates"].append([point_to_box[0], point_to_box[1], point_to_box[2]])


                # if self.tracked:
                #     input_dictionary["object_ids"] = list(self.tracked.tracker_id)
                # else:
                #     input_dictionary["object_ids"] = []

                self.user_message = str(input_dictionary)

                self.publish()

    


    def publish(self):

        if self.step == 2:
            self.transcribe = False
            self.prevent_loopback(self.transcribe)

            sleep(2)

            actor.talker("Go in front of the table with the green plant on it and wait there.")

            sleep(2)

            self.audio_queue.get(block=False) if not self.audio_queue.empty() else None
            self.transcribe = True
            self.prevent_loopback(self.transcribe)

        if self.step == 3:
            self.transcribe = False
            self.prevent_loopback(self.transcribe)

            sleep(2)

            actor.talker("Take one of the tools in front of you and show it to me.")

            sleep(2)

            self.audio_queue.get(block=False) if not self.audio_queue.empty() else None
            self.transcribe = True
            self.prevent_loopback(self.transcribe)

        elif self.step == 4:
            self.transcribe = False
            self.prevent_loopback(self.transcribe)

            sleep(2)

            actor.talker("Go with the tool to the four blue boxes on the table and wait there.")

            sleep(2)

            self.audio_queue.get(block=False) if not self.audio_queue.empty() else None
            self.transcribe = True
            self.prevent_loopback(self.transcribe)

        if self.step == 5:
            self.transcribe = False
            self.prevent_loopback(self.transcribe)

            sleep(2)

            actor.talker("Show me the tool again. Afterwards, put it in the box that I will show you now.")

            sleep(2)

            self.audio_queue.get(block=False) if not self.audio_queue.empty() else None
            self.transcribe = True
            self.prevent_loopback(self.transcribe)

        elif self.step == 6:
            self.transcribe = False
            self.prevent_loopback(self.transcribe)

            sleep(2)

            actor.talker("Thank you for your help! Now go back to the yellow area behind me.")

            sleep(2)

            self.audio_queue.get(block=False) if not self.audio_queue.empty() else None
            self.transcribe = True
            self.prevent_loopback(self.transcribe)


    def add_msg_to_info_logger(self, msg):
        """
        Log a message at the info level.

        Args:
            msg (str): The message to log.
        """
        self.get_logger().info(msg)



    def transformations(self, to="human"):

        rotation_matrix = np.array([[1, 0, 0, 0],
                                    [0, 1, 0, 0],
                                    [0, 0, -1, 0],
                                    [0, 0, 0, 1]])

        box1_calculation = np.array(self.box1_position)
        box2_calculation = np.array(self.box2_position)
        box3_calculation = np.array(self.box3_position)
        box4_calculation = np.array(self.box4_position)


        human_position_matrix = np.array([[1, 0, 0, self.eyetracker_position[0]],
                                        [0, 1, 0, self.eyetracker_position[1]],
                                        [0, 0, 1, self.eyetracker_position[2]],
                                        [0, 0, 0,                   1]])
                
        human_inverse = np.linalg.inv(human_position_matrix)
        

        position_of_truck = np.array([[1, 0, 0, self.citi_position[0]],
                                    [0, 1, 0, self.citi_position[1]],
                                    [0, 0, 1, self.citi_position[2]],
                                    [0, 0, 0,                   1]])
        
        position_of_box = np.array([[1, 0, 0, self.box4_position[0]],
                                    [0, 1, 0, self.box4_position[1]],
                                    [0, 0, 1, self.box4_position[2]],
                                    [0, 0, 0,                   1]])
        
        box_inverse = np.linalg.inv(position_of_box)

        
        position_of_tools = np.array([[1, 0, 0, self.tool2_position[0]],
                                    [0, 1, 0, self.tool2_position[1]],
                                    [0, 0, 1, self.tool2_position[2]],
                                    [0, 0, 0,                   1]])
        
        tools_inverse = np.linalg.inv(position_of_tools)



        centroid = (box1_calculation + box2_calculation + box3_calculation + box4_calculation) / 4

        position_of_boxes_combined = np.array([[1, 0, 0, centroid[0]],
                                    [0, 1, 0, centroid[1]],
                                    [0, 0, 1, centroid[2]],
                                    [0, 0, 0,           1]])
        
        position_of_boxes_combined_inverse = np.linalg.inv(position_of_boxes_combined)

        
        if to == "human":
            human_to_nao =  rotation_matrix @ (human_inverse @ position_of_truck)
            human_position_relative_to_nao = human_to_nao[0:3, 3].tolist()
            return human_position_relative_to_nao
        
        elif to == "box":
            box_to_nao = rotation_matrix @ (box_inverse @ position_of_truck)
            box_position_relative_to_nao = box_to_nao[0:3, 3].tolist()
            return box_position_relative_to_nao
        
        elif to == "tools":
            tools_to_nao = rotation_matrix @ (tools_inverse @ position_of_truck)
            tools_position_relative_to_nao = tools_to_nao[0:3, 3].tolist()
            return tools_position_relative_to_nao
        
        elif to == "boxes":
            boxes_to_nao = rotation_matrix @ (position_of_boxes_combined_inverse @ position_of_truck)
            boxes_position_relative_to_nao = boxes_to_nao[0:3, 3].tolist()
            return boxes_position_relative_to_nao

def main(args=None):
    """
    Initializes the ROS 2 Python client library, creates a ResponsePublisher node, logs a message, 
    and spins the node to keep it alive.

    Args:
        args (list, optional): Command line arguments. Defaults to None.
    """
    global actor
    actor = ActionGeneration()

    rclpy.init(args=args)

    framework = MergedInputPublisher()

    try:
        rclpy.spin(framework)

    except KeyboardInterrupt:
        pass

    finally:
        actor.posture.applyPosture("Sit", 1.0)

        #Destroy the node explicitly
        #(optional - otherwise it will be done automatically
        #when the garbage collector destroys the node object)
        framework.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()