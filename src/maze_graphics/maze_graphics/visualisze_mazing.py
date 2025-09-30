import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time

class MazeVisualizer(Node):
    def __init__(self):
        super().__init__('maze_visualizer')
        self.subscriber_ = self.create_subscription(Point, 'step', self.listener_callback, 10)
        self.current_pose = Point()
        self.current_pose.x = 0.0
        self.current_pose.y = 0.0
        self.current_pose.z = 0.0

    def listener_callback(self, msg):
        self.current_pose = msg
        self.get_logger().info(f'Received: x={self.current_pose.x}, y={self.current_pose.y}, direction={self.current_pose.z}')

def main(args=None):
    rclpy.init(args=args)
    maze_visualizer = MazeVisualizer()
    rclpy.spin(maze_visualizer)
    maze_visualizer.destroy_node()
    rclpy.shutdown()