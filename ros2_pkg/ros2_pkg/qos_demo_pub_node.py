import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class QoSDemoNode(Node):
    def __init__(self):
        super().__init__('qos_demo_node')

        my_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # 신뢰성: Best Effort
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher_ = self.create_publisher(
            String, 
            'sensor_data', 
            qos_profile=my_qos_profile # 여기에 적용!
        )

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        msg = String()

        msg.data = 'hello'
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = QoSDemoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()