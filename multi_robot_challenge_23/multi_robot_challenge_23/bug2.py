import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool
from bug2_interfaces.srv import SetCustomBool
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
import math

class Bug2Controller(Node):
    def __init__(self):
        super().__init__("bug2")

        self.wall_follower_client = self.create_client(SetBool, 'wall_follower_service')
        self.go_to_point_client = self.create_client(SetCustomBool, 'go_to_point_service')
        self.wait_for_services()

        self.subscription_scan = self.create_subscription(LaserScan, "scan", self.clbk_laser, 10)
        self.subscription_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.target_position = Point(x=0.0, y=5.0, z=0.0)
        self.position = None
        self.start_position = Point(x=0.0, y=0.0, z=0.0)
        self.obstacle_detected = 100.0

        self.is_following_wall = False
        self.left_line_point = None

        self.req_wf = SetBool.Request()
        self.req_gtp = SetCustomBool.Request()

        self.timer = self.create_timer(0.1, self.control_loop) 

    def start_go_to_point_service(self):
        self.req_gtp.move_switch = True
        self.req_gtp.target_position = self.target_position
        self.future = self.go_to_point_client.call_async(self.req_gtp)

    def start_wall_follower_service(self):
        self.req_wf.data = True
        self.future = self.wall_follower_client.call_async(self.req_wf)

    def stop_go_to_point(self):
        self.req_gtp.move_switch = False
        self.future = self.go_to_point_client.call_async(self.req_gtp)

    def stop_wall_follower(self):
        self.req_wf.data = False
        self.future = self.wall_follower_client.call_async(self.req_wf)
    
    def distance_to_goal(self):
        if self.position is None:
            return float('inf')
        return math.sqrt((self.target_position.x - self.position.x) ** 2 + (self.target_position.y - self.position.y) ** 2)
    
    def distance_to_line(self):
        if self.position is None or self.start_position is None:
            return float('inf')
        
        A = self.target_position.y - self.start_position.y
        B = self.start_position.x - self.target_position.x
        C = A * self.start_position.x + B * self.start_position.y
        return abs(A * self.position.x + B * self.position.y + C) / math.sqrt(A ** 2 + B ** 2)
    
    def odom_callback(self, msg):
        self.position = msg.pose.pose.position 
        orientation_q = msg.pose.pose.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    def clbk_laser(self, msg):
        self.obstacle_detected = msg.ranges[0] < 0.3

    def wait_for_services(self):
        while not self.wall_follower_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for wall follower service...')
        while not self.go_to_point_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for go to point service...')

    def control_loop(self):
        if self.is_following_wall:
            if self.distance_to_line() < 0.1 and self.distance_to_goal() < self.left_line_point:
                self.is_following_wall = False
                self.stop_wall_follower()
                self.start_go_to_point_service()   
        else:
            if self.obstacle_detected:          
                self.is_following_wall = True
                self.left_line_point = self.distance_to_goal()
                self.stop_go_to_point()
                self.start_wall_follower_service()

        print(self.distance_to_line(),'+',self.distance_to_goal())
        

def main(args=None):
    rclpy.init(args=args)
    controller = Bug2Controller()

    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()