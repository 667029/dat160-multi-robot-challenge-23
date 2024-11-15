import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class BraitenbergController(Node):
    def __init__(self):
        super().__init__('braitenberg')
        
        self.sub = self.create_subscription(LaserScan, 'scan', self.clbk_laser, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.lidar_left_front = 100
        self.lidar_right_front = 100

        self.timer = self.create_timer(1.0, self.timer_callback)

    def clbk_laser(self, msg):
        self.lidar_left_front = msg.ranges[12]
        self.lidar_right_front = msg.ranges[348]

    def timer_callback(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.4
        vel_msg.angular.z = 0.0

        if(self.lidar_left_front < 2) and (self.lidar_right_front < 2):
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.5
            print('Obstacle detected, stopping and trying to resolve by turning left')

        elif((self.lidar_left_front / self.lidar_right_front) < 1) and (self.lidar_left_front < 2):
            vel_msg.angular.z = -0.5
            print('Obstacle detected, turning right. Distance : "%s"' %self.lidar_left_front)

        elif((self.lidar_left_front / self.lidar_right_front) > 1) and (self.lidar_right_front < 2):
            vel_msg.angular.z = 0.5
            print('Obstacle detected, turning left. Distance : "%s"' %self.lidar_right_front)
        

        self.pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)

    controller = BraitenbergController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()