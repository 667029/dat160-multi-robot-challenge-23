import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64, String
from tf_transformations import euler_from_quaternion
import random
import math

class ExplorationNode(Node):
    def __init__(self):
        super().__init__('exploration_node')

        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        #self.pub_position = self.create_publisher(PoseStamped, 'robot_position', 10)

        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.sub_marker_id = self.create_subscription(Int64, 'marker_id', self.clbk_id, 10)
        self.sub_target_pos = self.create_subscription(Point, 'navigate_to_marker', self.clbk_target_pos, 10)
        self.sub_target_robot = self.create_subscription(String, 'target_robot', self.clbk_target_robot, 10)

        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.lidar_threshold = 0.7

        self.position = PoseStamped()
        self.target_position = None
        self.active_target = False

        self.state = 'Exploring'

        self.timer_control = self.create_timer(0.5, self.control_loop)
        self.timer_position = self.create_timer(1.0, self.publish_position)
   
    def publish_position(self):
       self.pub_position.publish(self.position)

    def clbk_id(self, msg):
        self.marker_id = msg.data

    def clbk_target_pos(self, msg):
        if self.active_target:
            self.target_position = msg
            self.get_logger().info(f'Recived target position')

    def clbk_target_robot(self, msg):
        if msg.data == self.get_name():
            self.active_target = True
            self.get_logger().info(f'Active')

    def laser_callback(self, msg):
        self.front_distance = min(msg.ranges[0:15] + msg.ranges[-15:])
        self.left_distance = min(msg.ranges[15:90])
        self.right_distance = min(msg.ranges[-90:-15])

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

    def angle_to_target(self):
        if not self.active_target or self.target_position is None:
            return
        
        dx = self.target_position.x - self.position.pose.position.x
        dy = self.target_position.y - self.position.pose.position.y
        return math.atan2(dy, dx)
    
    def distance(self):
        if not self.active_target or self.target_position is None:
            return

        dx = self.target_position.x - self.position.pose.position.x
        dy = self.target_position.y - self.position.pose.position.y
        return math.sqrt(dx ** 2 + dy ** 2)


    def navigate_to_goal(self):
        if not self.active_target or self.target_position is None:
            return
        
        twist = Twist()
        dx = self.target_position.x - self.position.pose.position.x
        dy = self.target_position.y - self.position.pose.position.y
        distance = math.sqrt(dx ** 2 + dy ** 2)

        if distance > 0.5:  # Fortsett mot målet
            angle_to_target = math.atan2(dy, dx)
            twist.linear.x = 0.2
            twist.angular.z = 0.3 * angle_to_target
        else:  # Mål nådd
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Reached target position!")
            self.active_target = False  # Deaktiver mål
        self.pub_cmd_vel.publish(twist)

    def target_navigation(self, angle_to_target):
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0.3 * angle_to_target
        self.pub_cmd_vel.publish(twist)

    def random_walking(self):
        twist = Twist()
        twist.linear.x = 0.3
        twist.angular.z = random.uniform(-0.5, 0.5)
        self.pub_cmd_vel.publish(twist)

    def stopp_movement(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.active_target = False
        self.pub_cmd_vel.publish(twist)

    def obstacle_avoidence(self):
        twist = Twist()
        if self.left_distance < self.lidar_threshold:
            twist.angular.z -= 0.7
        if self.right_distance < self.lidar_threshold:
            twist.angular.z += 0.7 
        self.pub_cmd_vel.publish(twist)


    def control_loop(self):
        twist = Twist()


        twist.linear.x = 0.0
        twist.angular.z = 0.0 

        # if self.state == 'Dispersing':


        #     else:
        #         self.state = 'Exploring'

        # elif self.state == 'Exploring':
        #     if self.front_distance < 1.0:
        #         vel_msg.linear.x = 0.0
        #         vel_msg.angular.z = 0.5 if self.left_distance > self.right_distance else -0.5
        #     else:
        #         vel_msg.linear.x = 0.3
        #         vel_msg.angular.z = random.uniform(-0.5, 0.5)

        # elif self.state == 'Still':
        #     if self.front_distance < 1.0:
        #         vel_msg.linear.x = 0.0
        #         vel_msg.angular.z = 0.0 


        self.pub_cmd_vel.publish(twist)

    def publish_position(self):
       self.pub_position.publish(self.position)

def main(args=None):
    rclpy.init(args=args)
    controller = ExplorationNode()

    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()