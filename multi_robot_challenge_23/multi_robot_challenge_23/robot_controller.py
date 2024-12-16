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
        self.sub_positions = self.create_subscription(PoseStamped, '/robot_positions', self.positions_callback, 10)
        self.sub_big_fire = self.create_subscription(Bool, 'big_fire', self.big_fire_callback, 10)

        self.radius = 5.0

        self.position = PoseStamped()
        self.neighbors = {}

        self.yaw = 0
        self.big_fire = False
        
        self.lidar_front = 100.0
        self.lidar_left_front = 100.0
        self.lidar_right_front = 100.0
        
        self.timer = self.create_timer(0.5, self.control_loop)

    def laser_callback(self, msg):
        self.lidar_front = msg.ranges[0]
        self.lidar_left_front = msg.ranges[12]
        self.lidar_right_front = msg.ranges[348]

    def positions_callback(self, msg):
        if msg.header.frame_id[-1:] != self.get_name()[-1:]:
            self.neighbors[msg.header.frame_id] = msg.pose.position

        if msg.header.frame_id[-1:] == self.get_name()[-1:]:
            self.position.pose.position = msg.pose.position
            orientation_q = msg.pose.orientation
            quaternion = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
            euler = euler_from_quaternion(quaternion)
            self.yaw = euler[2]
        
        self.update_neighbors()

    @staticmethod
    def calculate_distance(pos1, pos2):
        return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2)

    def big_fire_callback(self, msg):
        self.big_fire = msg.data

    def update_neighbors(self):
        new_neighbors = {}

        for robot_id, pos in self.neighbors.items():
            distance = self.calculate_distance(self.position.pose.position, pos)
            if distance <= self.radius:
                new_neighbors[robot_id] = pos

        self.neighbors = new_neighbors

    def avoid_obstacle(self, twist):
        if(self.lidar_left_front < 1.5) and (self.lidar_right_front < 1.5):
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            return True
        
        elif((self.lidar_left_front / self.lidar_right_front) < 0.5) and (self.lidar_left_front < 1.5):
            twist.angular.z = -0.5
            return True
        
        elif((self.lidar_left_front / self.lidar_right_front) > 0.5) and (self.lidar_right_front < 1.5):
            twist.angular.z = 0.5
            return True
        return False


    def rotate_to_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def control_loop(self):
        twist = Twist()

        if self.big_fire:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub_cmd_vel.publish(twist)
            return

        
        if self.avoid_obstacle(twist):
            self.pub_cmd_vel.publish(twist)
            return

        for robot_id, pos in self.neighbors.items():
            dx = self.position.pose.position.x - pos.x
            dy = self.position.pose.position.y - pos.y
            angle = math.atan2(dy, dx)
            target_angle = angle - self.yaw
            distance = self.calculate_distance(self.position.pose.position, pos)

            if(distance < 1.0) and ('0' != self.get_name()[-1:]):
                twist.linear.x = 0.4
                twist.angular.z = self.rotate_to_angle(target_angle)
                self.pub_cmd_vel.publish(twist)
                return
            elif(distance > 3.0) and ('0' != self.get_name()[-1:]):
                twist.linear.x = 0.4
                twist.angular.z = self.rotate_to_angle((target_angle + math.pi))
                self.pub_cmd_vel.publish(twist)
                return

        twist.linear.x = 0.4
        twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    controller = RobotControllerNode()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()