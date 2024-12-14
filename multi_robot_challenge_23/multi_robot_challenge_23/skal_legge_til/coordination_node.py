import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Pose, Point
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from std_msgs.msg import String
import math
import random

class CoordinationNode(Node):
    def __init__(self):
        super().__init__('coordination_node')

        self.pub_target_robot = self.create_publisher(String, 'target_robot', 10)
        self.pub_target_position = self.create_publisher(Point, 'navigate_to_marker', 10)

        self.sub_marker_position = self.create_subscription(Point, 'marker_position', self.marker_callback, 10)
        self.sub_robot_positions = self.create_subscription(PoseStamped, 'robot_positions', self.robot_position_callback, 10)

        self.marker_position = None  # Markørens posisjon
        self.robot_positions = {}  # {robot_name: PoseStamped}
        

    def marker_callback(self, msg):
        self.marker_position = msg
        self.get_logger().info(f"Marker position received: ({msg.x}, {msg.y})")
        self.assign_robot_to_marker()

    def robot_position_callback(self, msg):
        robot_name = msg.header.frame_id  
        self.robot_positions[robot_name] = msg

    def assign_robot_to_marker(self):
        if self.marker_position is None or not self.robot_positions:
            return

        nearest_robot = None
        min_distance = float('inf')

        # Finn roboten med minst avstand til markøren
        for robot_name, robot_pose in self.robot_positions.items():
            distance = self.calculate_distance(robot_pose.pose.position, self.marker_position)
            if distance < min_distance:
                nearest_robot = robot_name
                min_distance = distance

        if nearest_robot:
            self.get_logger().info(f"Assigning robot {nearest_robot} to marker at {self.marker_position}")
            # Publiser kommandoer
            self.pub_target_robot.publish(String(data=nearest_robot))
            self.pub_target_position.publish(self.marker_position)


    @staticmethod
    def calculate_distance(pos1, pos2):
        return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2)
    


def main(args=None):
    rclpy.init(args=args)
    node = CoordinationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
