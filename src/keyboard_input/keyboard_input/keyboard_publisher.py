import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from getkey import getkey

import sys
import os
sys.path.append(os.path.abspath(os.getcwd()))

from src.modules.action_module import ActionGeneration

class KeyboardPublisher(Node):
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
        super().__init__("KeyboardPublisher")
        self.keyboard_publisher = self.create_publisher(String, 'key_input', 10)

        self.create_timer = (0.05, self.key_publish)
        self.tracking = False
        self.blocking = False
        
        global activity
        activity = ActionGeneration()

    def key_publish(self):
        """
        Publish a given OpenCV image as a ROS Image message.

        Args:
            cv2_image (numpy.ndarray): The OpenCV image to be published.

        Raises:
            CvBridgeError: If conversion from OpenCV image to ROS Image message fails.
        """
        key = getkey()
        
        if key:
            msg = String()
            msg.data = key

            if key == "t":
                self.tracking = not self.tracking

            # Set object tracking on/off - basically to detect new objects
            if self.tracking == True and key == "t":
                self.add_msg_to_info_logger("Tracking on")
            elif self.tracking == False and key == "t":
                self.add_msg_to_info_logger("Tracking off")


            # Functionality to switch languages of the robot's answers - not implemented yet
            # if key == "s":
            #     self.add_msg_to_info_logger("Setting language to Swedish")
            # elif key == "e":
            #     self.add_msg_to_info_logger("Setting language to English")

            if key == "b":
                self.blocking = not self.blocking


            # Manually block audio transcriptions - basically to prevent the robot to respond to "unwanted" transcriptions
            if self.blocking == True and key == "b":
                self.add_msg_to_info_logger("Blocking transcription")
                activity.thinking()
            elif self.blocking == False and key == "b":
                self.add_msg_to_info_logger("Allowing transcription")
                activity.listening()


            self.keyboard_publisher.publish(msg)
        

    def add_msg_to_info_logger(self, msg):
        """
        Log a message at the info level.

        Args:
            msg (str): The message to log.
        """
        self.get_logger().info(msg)
        
def main(args=None):
    """
    The main entry point for the script. Initializes the ROS2 node, captures rgb video from the Kinect,
    and publishes the video frames as ROS Image messages until 'q' is pressed.

    Args:
        args (list, optional): Command-line arguments passed to the script.
    """
    rclpy.init(args=args)

    # Start publisher
    publisher = KeyboardPublisher()
    publisher.add_msg_to_info_logger("Listening now to keyboard inputs...")
    publisher.add_msg_to_info_logger("These are the possible key inputs:")
    publisher.add_msg_to_info_logger("b - block transmission ON/OFF")
    publisher.add_msg_to_info_logger("t - tracking ON/OFF")

    
    while True:
        try:
           publisher.key_publish() 

        except KeyboardInterrupt:
            break



    # closing  
    publisher.destroy_node()
    rclpy.shutdown()   

if __name__ == "__main__":
    main()
