import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import ffmpeg
import subprocess
import json
import selectors

# RTSP URL of the stream
RTSP_URL = 'rtsp://192.168.75.51:8554/live/all'

# FFmpeg command to filter out the specific payload (99 for gaze data)
ffmpeg_command = (
    ffmpeg
    .input(RTSP_URL)
    .output('pipe:', format='rawvideo', vcodec='copy', map='0:3', **{'an': None, 'vn': None})  # 0:3 corresponds to the gaze data stream
)

# Opening a subprocess for the RTSP stream
process = subprocess.Popen(
    ffmpeg_command.compile(),
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE
)

class GazeCoordinatesPublisher(Node):
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
        super().__init__("GazeCoordinatesPublisher")
        self.add_msg_to_info_logger("initializing gaze publisher")
        self.gaze_publisher = self.create_publisher(String, 'gaze_coordinates', 10)
        self.buffer = ''

        
    def publish(self, gaze2d):
        """
        Publish a given OpenCV image as a ROS Image message.

        Args:
            gaze3d (numpy.ndarray): The coordinates to be published in a [x, y, z] Format.

        """

        msg = String()
        msg.data = str(gaze2d)

        self.gaze_publisher.publish(msg)
        
        

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
    and publishes the gaze coordinates as ROS String message.

    Args:
        args (list, optional): Command-line arguments passed to the script.
    """
    rclpy.init(args=args)

    gaze_publisher = GazeCoordinatesPublisher()
    buffer = ''

    # Read and process the gaze data stream
    while True:
        try:
            in_bytes = process.stdout.read(1024)
            if not in_bytes:
                continue

            buffer += in_bytes.decode('utf-8', errors='ignore')


            while True:
                try:
                    data, idx = json.JSONDecoder().raw_decode(buffer)
                    buffer = buffer[idx:].lstrip()
                    
                    # Extract the "gaze3d" data
                    gaze2d = data.get('gaze2d')
                    if gaze2d is not None:
                        gaze_publisher.add_msg_to_info_logger(str(gaze2d))
                        gaze_publisher.publish(gaze2d)

                except json.JSONDecodeError:
                    # Break if the buffer does not contain a complete JSON object yet
                    break
        except KeyboardInterrupt:
            # closing  
            gaze_publisher.destroy_node()
            rclpy.shutdown()   
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
