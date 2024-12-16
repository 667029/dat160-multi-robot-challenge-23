import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose
from scoring_interfaces.srv import SetMarkerPosition
from std_msgs.msg import Int64, String, Bool
from geometry_msgs.msg import PointStamped, PoseStamped
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import random
import math

class MarkerDetectionNode(Node):
    def __init__(self):
        super().__init__('marker_detection_node')
        
        self.client = self.create_client(SetMarkerPosition, '/set_marker_position')
        self.wait_for_services()

        self.pub_big_fire = self.create_publisher(Bool, 'big_fire', 10)
        
        self.sub_bool_fire = self.create_subscription(Bool, 'bool_fire', self.bool_fire_callback, 10)
        self.sub_marker_id = self.create_subscription(Int64, 'marker_id', self.marker_id_callback, 10)
        self.sub_marker_pos = self.create_subscription(Pose, 'marker_map_pose', self.marker_position_callback, 10)
        self.sub_robot_position = self.create_subscription(PoseStamped, '/robot_positions', self.robot_position_callback, 10)

        self.marker_id = -1
        self.robot_pos = PoseStamped()
        self.marker_pos = Point()
        self.pos_accuracy = 1.5
        self.distance = None
        self.success_report = False
        self.remaining_time = 10
        self.req_msg = False

        self.timer = self.create_timer(1.0, self.control_loop)

    def marker_id_callback(self, msg):
        if -1 < msg.data < 6:
            self.marker_id = msg.data

    def marker_position_callback(self, msg):
        self.marker_pos = msg.position

    def robot_position_callback(self, msg):
        if msg.header.frame_id[-1:] == self.get_name()[-1:]:
            self.robot_pos.pose.position = msg.pose.position

    def bool_fire_callback(self, msg):
        self.success_report = msg.data
    
    def proximity_to_marker(self):
        self.distance = self.calculate_distance(self.robot_pos.pose.position, self.marker_pos)
        return self.pos_accuracy > self.distance

    @staticmethod    
    def calculate_distance(pos1, pos2):
        return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2)

    def req_help(self, data):
        msg = Bool()
        msg.data = data
        self.pub_big_fire.publish(msg)

            
    def control_loop(self):
        #self.get_logger().info('ID: ' + str(self.success_report))
        if self.proximity_to_marker() and self.marker_id == 4:
            if self.success_report is False:
                self.req_help(self.req_msg)
            elif self.success_report:
                self.req_msg.data = True
                self.req_help(self.req_msg)


        if self.marker_id == -1:
            return
        
        #self.get_logger().info('ID: ' + str(self.success_report))
        
        if self.proximity_to_marker():
            self.client_service()

    def client_service(self):
        self.request = SetMarkerPosition.Request()
        self.request.marker_id = self.marker_id
        self.request.marker_position = self.marker_pos
        self.future = self.client.call_async(self.request)
       
        if self.future.result() is False:
            self.get_logger().info('Reporting Failed')
        else:
            self.get_logger().info('Reporting Success')

    def wait_for_services(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again..')
        self.get_logger().info('Connected to /set_marker_position service.')


                


def main(args=None):
    rclpy.init(args=args)
    controller = MarkerDetectionNode()

    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()