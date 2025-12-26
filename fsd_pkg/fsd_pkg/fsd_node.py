import rclpy as rp 
from rclpy.node import Node 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np 
from ultralytics import YOLO
import cv2

class Fsd(Node):
    def __init__(self):
        super().__init__('fsd_node')
        model = YOLO("yolov8n.pt")
        self.subscriber_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.publisher_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.front_dist = 3.5
        self.left_dist = 3.5
        self.right_dist = 3.5
        self.backward_dist = 3.5
        self.safe_dist = 1.4

        # self.cap = cv2.VideoCapture(0)
        self.model = YOLO("yolov8n.pt")

    def scan_callback(self, msg):
        ranges = np.array([v if v is not None else np.nan for v in msg.ranges])
        ranges = np.where(np.isnan(ranges) | np.isinf(ranges), 3.5, ranges)

        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        front_mask_pos = (angles >= 0) & (angles <= np.pi/4)
        front_mask_neg_equiv = (angles >= 7*np.pi/4) & (angles <= msg.angle_max) 
        self.front_mask = front_mask_pos | front_mask_neg_equiv
        self.left_mask = (angles > np.pi/4) & (angles <= 3*np.pi/4) 
        self.backward_mask = (angles > 3*np.pi/4) & (angles <= 5*np.pi/4) 
        self.right_mask = (angles > 5*np.pi/4) & (angles <= 7*np.pi/4)

        if np.any(self.front_mask):
            self.front_dist = np.min(ranges[self.front_mask])
        else:
            self.front_dist = 3.5

        if np.any(self.left_mask):
            self.left_dist = np.min(ranges[self.left_mask])
        else:
            self.left_dist = 3.5

        if np.any(self.right_mask):
            self.right_dist = np.min(ranges[self.right_mask])
        else:
            self.right_dist = 3.5
        
        if np.any(self.backward_mask):
            self.backward_dist = np.min(ranges[self.backward_mask])
        else:
            self.backward_dist = 3.5

        self.driving(msg)        

    def trans_ranges(self, ranges):
        ranges = np.array([v if v is not None else np.nan for v in ranges])
        ranges = np.where(np.isnan(ranges) | np.isinf(ranges), 3.5, ranges)

        return ranges

    def driving(self, msg):
        driving_msg = Twist()
        safe = self.safe_dist

        # 1. 오른쪽 벽을 감지 (Wall Follower Logic)
        
        # 오른쪽에 벽이 없는 경우 (오른쪽으로 탈출 우선!)
        if self.right_dist > safe * 1.5:  # 안전거리보다 넉넉하게 열려 있을 때
            driving_msg.linear.x = 0.5  # 전진
            driving_msg.angular.z = -0.5 # 오른쪽으로 회전 (우회전)
            self.get_logger().warn("👉 오른쪽이 열림: 우회전하여 벽 따라가기 시작")
        
        # 전방에 벽이 없으면 직진 (오른쪽 벽을 적당히 따라가는 중)
        elif self.front_dist > safe:
            driving_msg.linear.x = 1.0  # 직진 속도 증가
            driving_msg.angular.z = 0.0
            self.get_logger().warn("🚀 직진")

        # 2. 전방이 막힌 경우 또는 오른쪽 벽을 잃었을 경우
        else:
            driving_msg.linear.x = 0.0 # 정지 후 회전 준비
            
            # 막다른 길: 전방과 우측이 모두 막혔을 때만 좌회전 (Right Wall Follower의 핵심)
            if self.front_dist <= safe and self.right_dist <= safe and self.left_dist > safe:
                driving_msg.angular.z = 0.5 # 좌회전
                self.get_logger().warn("⬅️ 막다른 길 또는 좁은 통로: 좌회전")
                
            # 전방이 막히고 좌우 모두 막혔을 때 (비상 상황)
            else:
                if self.backward_dist > safe:
                    driving_msg.linear.x = -0.5
                    driving_msg.angular.z = 1.0
                    self.get_logger().warn("🛑 모두 막힘, 후진")
                else:
                    self.get_logger().error("🚨 사방이 막힘, 정지")
                    driving_msg.linear.x = 0.0
                    driving_msg.angular.z = 0.0

        self.publisher_cmd.publish(driving_msg)


def main(args=None):
    rp.init(args=args)
    node = Fsd()
    rp.spin(node)
    node.destroy_node()
    rp.shutdown()
if __name__ == '__main__':
    main()

