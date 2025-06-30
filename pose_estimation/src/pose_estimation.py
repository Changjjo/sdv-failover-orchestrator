# mediapipe_pose_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import mediapipe as mp

class MediapipePoseNode(Node):
    def __init__(self):
        super().__init__('mediapipe_pose_node')
        self.get_logger().info("Mediapipe Pose Node has started.")
        
        # Initialize cv_bridge
        self.bridge = CvBridge()
        # Subscribe to image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/raw_image',
            self.image_callback,
            10
        )
        
        # Initialize Mediapipe Pose module
        self.mp_pose = mp.solutions.pose
        # Create Pose object with default parameters
        self.pose = self.mp_pose.Pose()
        self.get_logger().info("Mediapipe Pose object created.")
        # Initialize drawing utilities (for drawing landmarks on image)
        self.mp_drawing = mp.solutions.drawing_utils
        
        # Log additional info (for debugging purposes)
        self.get_logger().info("INFO: Created TensorFlow Lite XNNPACK delegate for CPU.")

    def image_callback(self, msg):
        self.get_logger().info(f"Received image on /camera/raw_image with encoding: {msg.encoding}")
        try:
            # Convert ROS Image message to OpenCV image (BGR8 encoding)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.get_logger().info("Image conversion to OpenCV format successful.")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        if cv_image is None:
            self.get_logger().error("Converted image is None!")
            return

        # Convert BGR image to RGB for Mediapipe processing
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        self.get_logger().debug("Converted BGR image to RGB.")

        # Process image with Mediapipe Pose
        self.get_logger().info("Processing image with Mediapipe Pose...")
        results = self.pose.process(rgb_image)
        self.get_logger().info("Pose processing complete.")

        # Check if pose landmarks are detected
        if results.pose_landmarks:
            self.get_logger().info("Pose landmarks detected. Drawing landmarks on image.")
            self.mp_drawing.draw_landmarks(cv_image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
        else:
            self.get_logger().warn("No pose landmarks detected in the image.")

        # Display the result image
        cv2.imshow("Pose Estimation", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MediapipePoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

