import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

class WallFollowerController(Node):
    def __init__(self):
        super().__init__("wallFollower")

        self.sub = self.create_subscription(LaserScan, "scan", self.clbk_laser, 10)
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
    
        self.lidar_front = 100.0
        self.lidar_left_front = 100.0
        self.lidar_right_front = 100.0

        self.mode = 'FOLLOW_WALL'

        self.active = False
        self.srv = self.create_service(SetBool, 'wall_follower_service', self.service_callback) 

        self.timer = self.create_timer(0.1, self.timer_callback) 

    def service_callback(self, request, response):
        self.active = request.data
        response.success = True
        return response

    def clbk_laser(self, msg):
        self.lidar_front = msg.ranges[0]  
        self.lidar_left_front = msg.ranges[45]
        self.lidar_right_front = msg.ranges[315]

    def timer_callback(self):
        if self.active is False:
            print('STOPPED')
            return

        msg = Twist()

        print(self.mode)

        d = 0.8
        
        if self.active:
            if self.lidar_front > d and self.lidar_left_front > d and self.lidar_right_front > d:
                self.mode = 'DEFAULT'
            
            elif self.lidar_front < d and self.lidar_left_front > d and self.lidar_right_front > d:
                self.mode = 'FRONT'
            
            elif self.lidar_front > d and self.lidar_left_front > d and self.lidar_right_front < d:
                self.mode = 'FRONT_RIGHT'
            
            elif self.lidar_front > d and self.lidar_left_front < d and self.lidar_right_front > d:
                self.mode = 'FRONT_LEFT'
            
            elif self.lidar_front < d and self.lidar_left_front > d and self.lidar_right_front < d:
                self.mode = 'FRONT_AND_FRONT_RIGHT'
            
            elif self.lidar_front < d and self.lidar_left_front < d and self.lidar_right_front > d:
                self.mode = 'FRONT_AND_FRONT_LEFT'
            
            elif self.lidar_front < d and self.lidar_left_front < d and self.lidar_right_front < d:
                self.mode = 'FRONT_AND_FRONT_LEFT_AND_FRONT_RIGHT'
            
            elif self.lidar_front > d and self.lidar_left_front < d and self.lidar_right_front < d:
                self.mode = 'FRONT_LEFT_AND_FRONT_RIGHT'
            
            else:
                self.mode = 'FAULT'


            if self.mode == 'DEFAULT':
                msg.linear.x = 0.3
                msg.angular.z = -0.5

            if self.mode == 'FRONT':
                msg.linear.x = 0.0
                msg.angular.z = 0.2

            if self.mode == 'FRONT_RIGHT':
                msg.linear.x = 0.3
                msg.angular.z = 0.1

            if self.mode == 'FRONT_LEFT':
                msg.linear.x = 0.0
                msg.angular.z = 0.3
            
            if self.mode == 'FRONT_AND_FRONT_RIGHT':
                msg.linear.x = -0.1
                msg.angular.z = 0.3
            
            if self.mode == 'FRONT_AND_FRONT_LEFT':
                msg.linear.x = -0.1
                msg.angular.z = 0.3

            if self.mode == 'FRONT_AND_FRONT_LEFT_AND_FRONT_RIGHT':
                msg.linear.x = -0.1
                msg.angular.z = 0.3

            if self.mode == 'FRONT_LEFT_AND_FRONT_RIGHT':
                msg.linear.x = -0.1
                msg.angular.z = 0.3

            self.pub.publish(msg)
        else:
            msg.linear.x = 0.0
            msg.angular.y = 0.0
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    controller = WallFollowerController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()