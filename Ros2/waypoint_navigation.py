import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class WaypointNav(Node):
    def __init__(self):
        super().__init__('waypoint_nav')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(5.0, self.send_goal)

    def send_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 1.0
        goal.pose.position.y = 2.0
        goal.pose.orientation.w = 1.0
        self.publisher.publish(goal)
        self.get_logger().info(f'Navigating to Waypoint: x={goal.pose.position.x}, y={goal.pose.position.y}')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
