import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class QoSDemoNode(Node):
    def __init__(self):
        super().__init__('qos_demo_node')

        my_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, # 신뢰성: Best Effort
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscriber_ = self.create_subscription(
            String, 
            'sensor_data', 
            self.sub_callback,
            qos_profile=my_qos_profile # 여기에 적용!
        )

    def sub_callback(self, msg):
        self.get_logger().info(f'msg ::: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = QoSDemoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()