import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Pose, Point
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import random

class PositionPublisherNode(Node):
    def __init__(self):
        super().__init__('position_publisher')

        self.pub_position = self.create_publisher(Point, 'robot_position', 10)

        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.timer= self.create_timer(1.0, self.publisher_controller)

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

    def publisher_controller(self):
        position_msg = Point()
        position_msg = self.position
        self.pub_position.publish(position_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PositionPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
