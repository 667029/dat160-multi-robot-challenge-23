import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64, String, Bool
from tf_transformations import euler_from_quaternion
import random
import math

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.sub_robot_position = self.create_subscription(Point, 'robot_position', self.robot_position_callback, 10)
        self.sub_big_fire = self.create_subscription(Bool, 'big_fire', self.big_fire_callback, 10)
        self.sub_bool_fire = self.create_subscription(Bool, 'bool_fire', self.bool_fire_callback, 10)

        self.position = Point()
        self.big_fire = False
        self.success_report = False
        
        
        self.timer = self.create_timer(0.5, self.control_loop)

    def laser_callback(self, msg):
        self.lidar_front = msg.ranges[0]
        self.lidar_left_front = msg.ranges[12]
        self.lidar_right_front = msg.ranges[348]

    def robot_position_callback(self, msg):
        self.robot_pos = msg

    def big_fire_callback(self, msg):
        self.big_fire = msg.data

    def bool_fire_callback(self, msg):
        self.success_report = msg.data

    def control_loop(self):
        twist = Twist()
        
        if self.big_fire:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        else:
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            
            # if(self.lidar_left_front < 1.5) and (self.lidar_right_front < 1.5):
            #     twist.linear.x = 0.0
            #     twist.angular.z = 0.5

            # elif((self.lidar_left_front / self.lidar_right_front) < 0.5) and (self.lidar_left_front < 1.5):
            #     twist.angular.z = -0.5

            # elif((self.lidar_left_front / self.lidar_right_front) > 0.5) and (self.lidar_right_front < 1.5):
            #     twist.angular.z = 0.5
    
        self.pub_cmd_vel.publish(twist)



def main(args=None):
    rclpy.init(args=args)
    controller = RobotControllerNode()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()