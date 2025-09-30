import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time
from .maze_gui import MazeWindow
from .maze_gui import Pose
from maze_interfaces.srv import Maze
import sys
from PyQt5.QtWidgets import QApplication

class MazeVisualizer(Node):
    def __init__(self):
        super().__init__('maze_visualizer')
        self.subscriber_ = self.create_subscription(Point, 'step', self.listener_callback, 10)
        self.current_pose = Point()
        self.current_pose.x = 0.0
        self.current_pose.y = 0.0
        self.current_pose.z = 0.0
        self.maze = None
        self.maze_client = self.create_client(Maze,"maze")
        while not self.maze_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.call_maze("small")

    def call_maze(self, maze_size):
        while not self.maze_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")
        request = Maze.Request()
        request.name = maze_size
        future = self.maze_client.call_async(request)
        future.add_done_callback(self.callback_maze_response)

    def callback_maze_response(self, future):
        response = future.result()
        self.maze = [response.maze[i*response.cols:(i+1)*response.cols] for i in range(response.rows)]
        
    def listener_callback(self, msg):
        self.current_pose = msg
        self.get_logger().info(f'Received: x={self.current_pose.x}, y={self.current_pose.y}, direction={self.current_pose.z}')
    
    def show_maze(self):
        app = QApplication(sys.argv)
        window = MazeWindow(self.maze)
        window.show()
        sys.exit(app.exec_())

def main(args=None):
    rclpy.init(args=args)
    maze_visualizer = MazeVisualizer()
    rclpy.spin(maze_visualizer)
    maze_visualizer.destroy_node()
    rclpy.shutdown()