import rclpy
from rclpy.node import Node
import math
import time

from tf_transformations import euler_from_quaternion
from gazebo_msgs.srv import GetEntityState
from nav_msgs.msg import Odometry
from scoring_interfaces.srv import SetMarkerPosition
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Int64, String, Bool

class CoordinationNode(Node):

    def __init__(self):
        super().__init__('coordination_node')
        self.inital_positions = [Point(), Point(), Point(), Point(), Point()]
        self.inital_orientations = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.robot_positions = [Point(), Point(), Point(), Point(), Point()]
        self.robot_orientations = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.robot_count = 2

        self.pub_ok_fire = self.create_publisher(Bool, '/tb3_0/bool_fire', 10)
        self.pub_ok_fire = self.create_publisher(Bool, '/tb3_1/bool_fire', 10)

        self.create_subscription(Odometry, '/tb3_0/odom', self.clbk_tb3_0_odom, 10)
        self.create_subscription(Odometry, '/tb3_1/odom', self.clbk_tb3_1_odom, 10)
        self.create_subscription(Odometry, '/tb3_2/odom', self.clbk_tb3_2_odom, 10)
        self.create_subscription(Odometry, '/tb3_3/odom', self.clbk_tb3_3_odom, 10)
        self.create_subscription(Odometry, '/tb3_4/odom', self.clbk_tb3_4_odom, 10)

        self.client_entity_state = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        while not self.client_entity_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.model_state_markers = []
        for i in range(5):
            new_model_state = self.get_model_state("Marker"+str(i))
            self.model_state_markers.append(new_model_state)

        for i in range(self.robot_count):
            while self.get_model_state("tb3_"+str(i)).state.pose.position.x == 0.0 and self.get_model_state("tb3_"+str(i)).state.pose.position.y == 0.0:
                self.get_logger().info("waiting for turtlebot to spawn")
                time.sleep(1)
            new_robot_state = self.get_model_state("tb3_"+str(i))
            self.inital_positions[i] = new_robot_state.state.pose.position
            self.inital_orientations[i] = self.get_yaw(new_robot_state.state.pose.orientation)

        self.timer = self.create_timer(0.5, self.timer_callback)
         
    def get_yaw(self, orientation):
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w)
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]
        return yaw

    def clbk_tb3_0_odom(self, msg):
        self.robot_positions[0] = msg.pose.pose.position
        self.robot_orientations[0] = self.get_yaw(msg.pose.pose.orientation)

    def clbk_tb3_1_odom(self, msg):
        self.robot_positions[1] = msg.pose.pose.position
        self.robot_orientations[1] = self.get_yaw(msg.pose.pose.orientation)
    
    def clbk_tb3_2_odom(self, msg):
        self.robot_positions[2] = msg.pose.pose.position
        self.robot_orientations[2] = self.get_yaw(msg.pose.pose.orientation)

    def clbk_tb3_3_odom(self, msg):
        self.robot_positions[3] = msg.pose.pose.position
        self.robot_orientations[3] = self.get_yaw(msg.pose.pose.orientation)

    def clbk_tb3_4_odom(self, msg):
        self.robot_positions[4] = msg.pose.pose.position
        self.robot_orientations[4] = self.get_yaw(msg.pose.pose.orientation)

    def get_model_state(self, model_name):
        req = GetEntityState.Request()
        req.name = model_name
        self.future = self.client_entity_state.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def check_big_fire_proximity(self):
        robot_cntr = 0
        for robot_pos in self.robot_positions:
            x_error = self.model_state_markers[4].state.pose.position.x - robot_pos.x
            y_error = self.model_state_markers[4].state.pose.position.y - robot_pos.y
            distance = math.sqrt(pow(x_error,2)+pow(y_error,2))
            if distance < 2.0:
                robot_cntr += 1

        if robot_cntr >= 2:
            return True

        return False
    
    def timer_callback(self):
        msg = Bool()
        msg.data = self.check_big_fire_proximity()
        self.pub_ok_fire.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    controller = CoordinationNode()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()