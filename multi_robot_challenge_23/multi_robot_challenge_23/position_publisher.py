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

        self.pub_positions = self.create_publisher(PoseStamped, '/robot_positions', 10)

        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.radius = 1.0

        self.position = PoseStamped()


    def odom_callback(self, msg):
        self.position.header.frame_id = self.get_name()
        self.position.header.stamp = self.get_clock().now().to_msg()
        self.position.pose.position = msg.pose.pose.position 
        self.position.pose.orientation = msg.pose.pose.orientation
        orientation_q = self.position.pose.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]
        self.pub_positions.publish(self.position)

def main(args=None):
    rclpy.init(args=args)
    node = PositionPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
