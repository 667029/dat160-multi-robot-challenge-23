import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from bug2_interfaces.msg import TargetPoint #Husk å legge til bug2_interfaces som <depend> i package.xml
from bug2_interfaces.srv import SetCustomBool #Husk å legge til bug2_interfaces som <depend> i package.xml
import math
from tf_transformations import euler_from_quaternion
from bug2_interfaces.srv import SetCustomBool

class GoToPointController(Node):

    def __init__(self):
        super().__init__('goToPoint')

        self.subscription_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.publisher_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.position = None
        self.yaw = None
        
        self.active = False
        self.goal = Point()
        
        self.srv = self.create_service(SetCustomBool, 'go_to_point_service', self.service_callback)  

        self.timer = self.create_timer(0.1, self.control_loop)

    def service_callback(self, request, response):
        self.active = request.move_switch
        self.goal = request.target_position
        response.success = True
        return response
    
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

    def control_loop(self):
        if self.position is None or self.yaw is None or self.active is False:
            print('STOPPED')
            return
        
        goal_vector = (self.goal.x - self.position.x, self.goal.y - self.position.y)
        goal_distance = math.sqrt(goal_vector[0] ** 2 + goal_vector[1] ** 2)
        goal_angle = math.atan2(goal_vector[1], goal_vector[0])
        angle_diff = goal_angle - self.yaw

        twist = Twist()
        
        if goal_distance > 0.2 and self.active: 
            twist.linear.x = 0.2
            twist.angular.z = 0.3 * angle_diff
            print('GOING_TO_POINT')
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            print('POINT_REACHED')
        
        self.publisher_cmd_vel.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    controller = GoToPointController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()