import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from math import pow, atan2, sqrt, sin, cos, pi
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan
import numpy as np
class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_param_node')

        # added part 1 : declare parameter
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.add_on_set_parameters_callback(self.parameter_callback)

        # added part 2 : initialize variable via parameter
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value

        self.lookahead_distance = 0.5
        self.linear_velocity = 0.2
        self.goal_tolerance = 0.2

        self.path = [
            [self.goal_x, self.goal_y]
        ]
        self.current_waypoint_index = 0

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription_ = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        self.subscription_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.is_localized = False

        self.timer = self.create_timer(0.5, self.control_loop)
        self.get_logger().info("Pure Pursuit Node Started! Waiting for AMCL pose...")

        self.front_dist = 0.0
        self.left_dist = 0.0
        self.right_dist = 0.0
        self.back_dist = 0.0

    def scan_callback(self, msg):
        ranges = np.array([x if x is not None else np.nan for x in msg.ranges], dtype=float)
        ranges = np.where(np.isnan(ranges) | np.isinf(ranges), 3.5, ranges)

        front = np.concatenate((ranges[315:360], ranges[0:45]))
        left = ranges[45:135]
        back = ranges[135:225]
        right = ranges[225:315]

        self.front_dist = np.mean(front).round(2)
        self.left_dist  = np.mean(left).round(2)
        self.right_dist = np.mean(right).round(2)
        self.back_dist = np.mean(back).round(2)

    # added part 3 : callback when parameter set
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'goal_x':
                self.goal_x = param.value
                self.get_logger().info(f'goal_x updated: {self.goal_x}')
            if param.name == 'goal_y':
                self.goal_y = param.value
                self.get_logger().info(f'goal_y updated: {self.goal_y}')

        self.path = [
            [self.goal_x, self.goal_y]
        ]
        self.current_waypoint_index = 0
        return SetParametersResult(successful=True)

    def pose_callback(self, msg):
        self.get_logger().info(f"pose callback : {type(msg)}")
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # formula to get current yaw (current yaw? angular.z degree!)
        q = msg.pose.pose.orientation
        self.current_yaw = atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.is_localized = True

    def control_loop(self):
        if not self.is_localized:
            return

        if self.current_waypoint_index >= len(self.path):
            self.stop_robot()
            return

        goal_x = self.path[self.current_waypoint_index][0]
        goal_y = self.path[self.current_waypoint_index][1]

        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance = sqrt(pow(dx, 2) + pow(dy, 2))

        if distance < self.goal_tolerance:
            self.get_logger().info(f"Waypoint {self.current_waypoint_index} Reached!")
            self.current_waypoint_index += 1
            return

        target_angle = atan2(dy, dx)
        alpha = target_angle - self.current_yaw

        if alpha > pi:
            alpha -= 2 * pi
        elif alpha < -pi:
            alpha += 2 * pi

        angular_velocity = self.linear_velocity * (2.0 * sin(alpha)) / distance

        cmd = Twist()

        self.get_logger().info(f"front_dist: {self.front_dist}")
        self.get_logger().info(f"left_dist: {self.left_dist}")
        self.get_logger().info(f"right_dist: {self.right_dist}")
        self.get_logger().info(f"back_dist: {self.back_dist}")

        safe_limit = 0.5 
        
        if self.front_dist < safe_limit:
            cmd.linear.x = 0.0
            if self.left_dist > self.right_dist:
                cmd.angular.z = 0.8 
                self.get_logger().info("정면 막힘: 왼쪽으로 회전 중")
            else:
                cmd.angular.z = -0.8 
                self.get_logger().info("정면 막힘: 오른쪽으로 회전 중")
        elif self.left_dist < safe_limit:
            cmd.linear.x = 0.1   
            cmd.angular.z = -0.4
            self.get_logger().info("왼쪽 벽 접근: 오른쪽으로 조향")
        elif self.right_dist < safe_limit:
            cmd.linear.x = 0.1  
            cmd.angular.z = 0.4
            self.get_logger().info("오른쪽 벽 접근: 왼쪽으로 조향")
        else:
            self.get_logger().info("경로 깨끗: 목적지로 순항 중")
            cmd.linear.x = self.linear_velocity
            cmd.angular.z = self.linear_velocity * (2.0 * sin(alpha)) / distance
            
        cmd.angular.z = max(min(cmd.angular.z, 1.2), -1.2)
        self.publisher_.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.publisher_.publish(cmd)
        self.get_logger().info("All waypoints completed. Robot Stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()