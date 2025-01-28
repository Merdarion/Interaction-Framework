import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridge, CvBridgeError
import pykinect_azure as pykinect
from pykinect_azure import K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_DEPTH, k4a_float2_t
from pykinect_azure import Calibration


# We need this function for the object-segmentation - it will be imported there to calculate the world coordinates
def get_3d_coordinates(depth_frame, cx, cy):
        rgb_depth = depth_frame[cx, cy]

        pixels = k4a_float2_t((cx, cy))

        pos3d_depth = camera.calibration.convert_2d_to_3d(pixels, rgb_depth, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_DEPTH)
        print(f"RGB depth: {rgb_depth}, Depth pos3D: {pos3d_depth}")

        return pos3d_depth.xyz.x, pos3d_depth.xyz.y, pos3d_depth.xyz.z


class KinectPublisher(Node):
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
        super().__init__("KinectPublisher")
        self.add_msg_to_info_logger("initializing node")
        self.rgb_publisher = self.create_publisher(Image, 'rgb_image', 10)
        self.depth_publisher = self.create_publisher(Image, 'depth_image', 10)
        self.bridge = CvBridge()
        
    def rgb_publish(self, rgb_frame):
        """
        Publish a given OpenCV image as a ROS Image message.

        Args:
            cv2_image (numpy.ndarray): The OpenCV image to be published.

        Raises:
            CvBridgeError: If conversion from OpenCV image to ROS Image message fails.
        """
        try:
            msg = Image()
            msg = self.bridge.cv2_to_imgmsg(rgb_frame, "bgr8")
            self.rgb_publisher.publish(msg)
        except CvBridgeError as e:
             self.add_msg_to_info_logger(e)
        #self.add_msg_to_info_logger("sending_image")

    def depth_publish(self, depth_frame):
        """
        Publish a given OpenCV image as a ROS Image message.

        Args:
            cv2_image (numpy.ndarray): The OpenCV image to be published.

        Raises:
            CvBridgeError: If conversion from OpenCV image to ROS Image message fails.
        """
        try:
            msg = Image()
            msg = self.bridge.cv2_to_imgmsg(depth_frame, "mono16")
            self.depth_publisher.publish(msg)
        except CvBridgeError as e:
             self.add_msg_to_info_logger(e)
        #self.add_msg_to_info_logger("sending_image soos")
        

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

    # Initialize the library, if the library is not found, add the library path as argument
    pykinect.initialize_libraries()

    # Modify camera configuration
    device_config = pykinect.default_configuration
    device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1080P
    device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED   
    
    # Start device
    global camera
    camera = pykinect.start_device(config=device_config)

    # Start publisher
    publisher = KinectPublisher()
    publisher.add_msg_to_info_logger("initializing camera")

    while True:
        capture = camera.update()

        rgb_ret, rgb_frame = capture.get_color_image()
        depth_ret, depth_frame = capture.get_transformed_depth_image()

        try:
            if rgb_ret:
                publisher.rgb_publish(rgb_frame)
            if depth_ret:
                publisher.depth_publish(depth_frame)

        except Exception as e:
            publisher.add_msg_to_info_logger(e)
            pass


        
        pressed = cv2.waitKey(1)

        if pressed == ord('q'):
        # closing application if esc or q pressed
            publisher.add_msg_to_info_logger("closing button pressed, closing")
            break

    # closing  
    publisher.destroy_node()
    rclpy.shutdown()   

if __name__ == "__main__":
    main()
