import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseStamped, Point, Pose
from std_msgs.msg import Int64, String
import math

class CommunicationNode(Node):
    def __init__(self):
        super().__init__('communication_node')

        # Publikasjoner
        #self.pub_marker_status = self.create_publisher(String, 'marker_status', 10)   
       
        # Abonnementer
        self.sub_marker_id = self.create_subscription(Int64, 'marker_id', self.clbk_id, 10)
        self.sub_marker_pos = self.create_subscription(Pose, 'marker_map_pose', self.clbk_marker_pos, 10)
        self.sub_positions = self.create_subscription(PoseStamped, 'robot_position', self.clbk_robot_pos, 10)

        # Posisjonsdata
        self.robot_pos = Point()
        self.marker_pos = Point()

        self.timer = self.create_timer(1.0, self.detect_marker)

    def clbk_id(self, msg):
        self.marker_id = msg.data

    def clbk_marker_pos(self, msg):
        self.marker_pos = msg.position

    def clbk_robot_pos(self, msg):
        self.robot_pos = msg.position
    
    def distance_to_goal(self):
        if self.robot_pos is None:
            return float('inf')
        return math.sqrt((self.marker_pos.x - self.robot_pos.x) ** 2 + (self.marker_pos.y - self.robot_pos.y) ** 2)
    
    def proximity_to_marker(self):
        if self.distance_to_goal() < 2:
            return True
        else:
            return False
    
    
 
    def marker_callback(self, msg):
        # Lagre ny markørposisjon
        self.markers.append((msg.point.x, msg.point.y))
        self.get_logger().info(f"New marker at: {msg.point.x}, {msg.point.y}")

    def position_callback(self, msg):
        # Oppdater robotens posisjon
        robot_name = msg.header.frame_id  # Antar robotnavn er i frame_id
        self.positions[robot_name] = (msg.pose.position.x, msg.pose.position.y)
        self.check_collaboration()

    def check_collaboration(self):
        # Sjekk om to roboter er nær en markør
        for marker in self.markers:
            nearby_robots = []
            for robot, position in self.positions.items():
                distance = math.sqrt((position[0] - marker[0])**2 + (position[1] - marker[1])**2)
                if distance < 1.5:  # Terskel for "nær markør"
                    nearby_robots.append(robot)

            if len(nearby_robots) >= 2:
                self.get_logger().info(f"Marker at {marker} confirmed by {nearby_robots}")
                self.markers.remove(marker)




def main(args=None):
    rclpy.init(args=args)
    controller = CommunicationNode()

    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()