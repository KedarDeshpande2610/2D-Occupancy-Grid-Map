import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class WallFollowingRobot(Node):

    def __init__(self):
        super().__init__('wall_following_robot')

        self.bridge = CvBridge()

        # Subscriptions to camera topics
        self.subscription2 = self.create_subscription(
            Image,
            '/overhead_camera/overhead_camera2/image_raw',
            self.listener_callback2,
            10)
        
        self.subscription4 = self.create_subscription(
            Image,
            '/overhead_camera/overhead_camera4/image_raw',
            self.listener_callback4,
            10)

        # Publisher to send velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Variables to store images
        self.image2 = None  # Bottom left (camera 2)
        self.image4 = None  # Top left (camera 4)

        # Timer to periodically process images and control robot
        self.timer = self.create_timer(1.0, self.control_robot)

        # Initialize variables for controlling robot
        self.map_image = None

    def listener_callback2(self, msg):
        self.image2 = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def listener_callback4(self, msg):
        self.image4 = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def control_robot(self):
        if self.image2 is not None and self.image4 is not None:
            self.process_images()
            self.follow_wall()

    def process_images(self):
        # Get individual image shapes
        h2, w2, _ = self.image2.shape
        h4, w4, _ = self.image4.shape

        # Determine the total dimensions for the combined image
        total_height = h2 + h4
        total_width = max(w2, w4)

        # Initialize the combined image with a black background
        combined_image = np.zeros((total_height, total_width, 3), dtype=np.uint8)

        # Paste images into the combined image with appropriate shifts
        combined_image[0:h4, 0:w4] = self.image4  # Top-left (image 4)
        combined_image[h4:h4+h2, 0:w2] = self.image2  # Bottom-left (image 2)

        # Convert to grayscale
        grayscale_image = cv2.cvtColor(combined_image, cv2.COLOR_BGR2GRAY)

        # Thresholding to binary image
        _, binary_image = cv2.threshold(grayscale_image, 128, 255, cv2.THRESH_BINARY)

        # Create or initialize the map image with white background
        if self.map_image is None or self.map_image.shape != grayscale_image.shape:
            self.map_image = np.ones_like(grayscale_image) * 255  # White background

        # Use binary image to highlight walls in the map image
        self.map_image[binary_image == 0] = 0  # Black pixels for detected walls

        # Save the map image periodically (e.g., every 10 seconds)
        if int(time.time()) % 10 == 0:
            cv2.imwrite('map.pgm', self.map_image)
            self.get_logger().info('Map saved as map.pgm')

    def follow_wall(self):
        # Example wall-following logic
        # Adjust linear and angular velocity based on sensor readings
        cmd_vel = Twist()

        # Example logic to turn left if the robot is too close to the wall
        if self.image4 is not None:
            left_wall_distance = np.mean(self.image4[:, -1])  # Simplified example
            if left_wall_distance < 100:  # Distance threshold
                cmd_vel.angular.z = 0.3  # Turn right
            else:
                cmd_vel.angular.z = 0.0  # Go straight

        cmd_vel.linear.x = 0.2  # Move forward

        # Publish velocity commands
        self.cmd_vel_publisher.publish(cmd_vel)

        self.get_logger().info(f'Published velocity command: Linear x: {cmd_vel.linear.x}, Angular z: {cmd_vel.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowingRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
