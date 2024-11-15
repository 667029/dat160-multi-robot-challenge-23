import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Int64
from scoring_interfaces.srv import SetMarkerPosition

class ReportMarkerController(Node):
    def __init__(self):
        super().__init__("ReportMarkerNode")

        self.client = self.create_client(SetMarkerPosition, '/set_marker_position')
        self.wait_for_services()
       
        self.sub_id = self.create_subscription(Int64, 'marker_id', self.clbk_id, 10)
        self.sub_pose = self.create_subscription(Pose, 'marker_map_pose', self.clbk_pos, 10)
        
        self.marker_id = -1
        self.marker_pose = Point()

        self.timer = self.create_timer(1.0, self.control_loop) 
   
    def clbk_id(self, msg):
        self.marker_id = msg.data

    def clbk_pos(self, msg):
        self.marker_pose = msg.position

    def client_service(self):
        self.request = SetMarkerPosition.Request()
        self.request.marker_id = self.marker_id
        self.request.marker_position = self.marker_pose
        self.future = self.client.call_async(self.request)

    def wait_for_services(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again..')
        self.get_logger().info('Connected to /set_marker_position service.')
    
    def control_loop(self):
        if -1 < self.marker_id < 5:
            self.client_service()

def main(args=None):
    rclpy.init(args=args)
    controller = ReportMarkerController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()