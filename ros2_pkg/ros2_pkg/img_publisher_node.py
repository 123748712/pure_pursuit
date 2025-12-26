import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np 
from ultralytics import YOLO
import cv2

class ImgPublisher(Node):
    def __init__(self):
        super().__init__('img_publisher_node')
        self.cv_bridge = CvBridge()
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,  # ← 여기!
            history=HistoryPolicy.KEEP_LAST
        )
        self.subscriber = self.create_subscription(Image, '/camera/image_raw', self.sub_callback, qos)
        self.publisher = self.create_publisher(Image, '/yolo/image_raw', 10)

        self.model = YOLO("digit_best.pt")
        self.seen_ids = set()
        self.class_count = {}

    def sub_callback(self, msg):
        frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        results = self.model.predict(
            frame_rgb,
            conf=0.3,
            imgsz=640,
            verbose=False
        )

        detected = False

        for r in results:
            if r.boxes is None:
                continue

            for box in r.boxes:
                detected = True

                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                conf = float(box.conf[0])

                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # 🔎 로그 출력 (중요)
                self.get_logger().info(
                    f'DETECTED digit={class_name}, conf={conf:.2f}'
                )

                # 박스 + 텍스트
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    frame,
                    f'{class_name} {conf:.2f}',
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

        if not detected:
            self.get_logger().info('NO DIGIT DETECTED')

        # OpenCV → ROS
        out_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out_msg.header = msg.header

        self.publisher.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImgPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
