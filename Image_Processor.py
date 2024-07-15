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
        self.subscription2 = self.create_subscription(Image, '/overhead_camera/overhead_camera2/image_raw', self.listener_callback2, 10)
        self.subscription4 = self.create_subscription(Image, '/overhead_camera/overhead_camera4/image_raw', self.listener_callback4, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.image2 = None
        self.image4 = None
        self.timer = self.create_timer(1.0, self.control_robot)
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
        h2, w2, _ = self.image2.shape
        h4, w4, _ = self.image4.shape
        total_height = h2 + h4
        total_width = max(w2, w4)
        combined_image = np.zeros((total_height, total_width, 3), dtype=np.uint8)
        combined_image[0:h4, 0:w4] = self.image4
        combined_image[h4:h4+h2, 0:w2] = self.image2

        grayscale_image = cv2.cvtColor(combined_image, cv2.COLOR_BGR2GRAY)
        _, binary_image = cv2.threshold(grayscale_image, 128, 255, cv2.THRESH_BINARY)

        if self.map_image is None or self.map_image.shape != grayscale_image.shape:
            self.map_image = np.ones_like(grayscale_image) * 255

        self.map_image[binary_image == 0] = 0

        if int(time.time()) % 10 == 0:
            cv2.imwrite('map.pgm', self.map_image)
            self.get_logger().info('Map saved as map.pgm')

    def follow_wall(self):
        cmd_vel = Twist()

        if self.image4 is not None:
            left_wall_distance = np.mean(self.image4[:, -1])
            if left_wall_distance < 100:
                cmd_vel.angular.z = 0.3
            else:
                cmd_vel.angular.z = 0.0

        cmd_vel.linear.x = 0.2

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
