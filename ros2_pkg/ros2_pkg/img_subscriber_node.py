import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

class ImgSubscriber(Node):
    def __init__(self):
        super().__init__('img_subscriber_node')
        self.cv_bridge = CvBridge()

        self.subscriber = self.create_subscription(Image, '/yolo/image_raw', self.sub_callback, 10)

    def sub_callback(self, msg):
        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("Remote Image", img)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = ImgSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()