import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time
from .maze_gui import MazeWidget
from .maze_gui import Pose
from maze_interfaces.srv import Maze
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QMainWindow
from threading import Thread
from rclpy.executors import MultiThreadedExecutor
from PyQt5.QtCore import QTimer

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
        self.maze_size_parameter_ = self.declare_parameter("maze_size","big")
        maze_size = self.get_parameter("maze_size").value
        self.call_maze(maze_size)

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


class MazeWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.setWindowTitle("Maze Robot Viewer")
        self.maze_widget = MazeWidget(node.maze)
        self.setCentralWidget(self.maze_widget)

        self.show()

        # Timer to refresh window
        self.timer = QTimer()
        self.timer.timeout.connect(lambda: self.maze_widget.move_robot(node.current_pose))
        self.timer.start(10)  # ms

def main(args=None):
    rclpy.init(args=args)

    hmi_node = MazeVisualizer()
    executor = MultiThreadedExecutor()
    executor.add_node(hmi_node)
    thread = Thread(target=executor.spin)
    thread.start()
    hmi_node.get_logger().info("Spinned ROS2 Node . . .")
    time.sleep(1)  # Wait a moment to ensure the maze is received
    app = QApplication(sys.argv)
    HMI = MazeWindow(hmi_node)
    # Start the ROS2 node on a separate thread
    

    # Let the app running on the main thread
    try:
        HMI.show()
        sys.exit(app.exec_())

    finally:
        hmi_node.get_logger().info("Shutting down ROS2 Node . . .")
        hmi_node.destroy_node()
        executor.shutdown()