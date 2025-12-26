import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from math import pow, atan2, sqrt, sin, cos, pi

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')


        '''
            teacher: 
            how to calculate angular velocity?
            linear_velocity * (2 * sin(diff_to_target_angle) / lookahead_distance)
        '''

        self.lookahead_distance = 0.5 # 전방 주시 거리 (훈련생이 정하기)
        self.linear_velocity = 0.2 # linear.x 값 (훈련생이 정하기)     
        self.goal_tolerance = 0.2 # 목표와의 거리 허용 범위 (훈련생이 정하기)     

        self.path = [
            [3.557,  -0.168]
        ]
        self.current_waypoint_index = 0

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription_ = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose', 
            self.pose_callback,
            10
        )
        
        # 현재 위치 
        self.current_x = 0.0
        self.current_y = 0.0
        # 현재 각도 (angular.z)
        self.current_yaw = 0.0
        
        self.is_localized = False

        self.timer = self.create_timer(0.5, self.control_loop)
        self.get_logger().info("Pure Pursuit Node Started! Waiting for AMCL pose...")

		# /amcl_pose 받으면 난 뭐할까?
    def pose_callback(self, msg):
        # self.get_logger().info(f"pose callback : {type(msg)}")
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # formula to get current yaw (current yaw? angular.z degree!)
        q = msg.pose.pose.orientation
        
        # 현재 각도 구하는 공식 
        self.current_yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.is_localized = True

		# 주행 명령을 발행하기 위한 함수 
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

				# 내가 생각하는 허용 범위 안에 들어왔다면, 난 도착이니까 그만해도 돼!
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

        # pure pursuit 핵심 공식 적용
        angular_velocity = self.linear_velocity * (2.0 * sin(alpha)) / distance

        cmd = Twist()
        cmd.linear.x = self.linear_velocity
        cmd.angular.z = angular_velocity
        
        if cmd.angular.z > 1.0: cmd.angular.z = 1.0
        if cmd.angular.z < -1.0: cmd.angular.z = -1.0
        
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