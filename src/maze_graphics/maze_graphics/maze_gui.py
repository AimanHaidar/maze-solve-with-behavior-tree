import sys
from PyQt5.QtWidgets import QApplication, QWidget, QMainWindow
from PyQt5.QtGui import QPainter, QPen, QColor, QBrush
from PyQt5.QtCore import Qt, QTimer

CELL_SIZE = 60
WALL_THICKNESS = 4

# Directions
UP, DOWN, LEFT, RIGHT = 0, 1, 2, 3
class Pose:
    def __init__(self, x=0, y=0, direction=UP):
        self.x = x
        self.y = y
        self.direction = direction

    def __eq__(self, other):
        return (self.x, self.y, self.direction) == (other.x, other.y, other.direction)


# Maze definition (1 = wall, 0 = free)
maze = [
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,2,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,3,1],
    [1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1],
    [1,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,1],
    [1,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,1,0,1],
    [1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1],
    [1,0,1,1,1,1,1,1,1,0,1,0,1,1,1,0,1,0,1],
    [1,0,0,0,0,0,0,1,0,0,1,0,0,0,0,0,1,0,1],
    [1,1,1,0,1,1,0,1,1,1,1,1,0,1,1,0,1,1,1],
    [1,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,1],
    [1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
]



# Robot path (sequence of positions)
path = [(1,1), (1,2), (1,3), (2,3), (3,3), (3,2), (3,1)]

class MazeWidget(QWidget):
    def __init__(self, maze):
        super().__init__()
        self.maze = maze
        self.rows = len(maze)
        self.cols = len(maze[0])
        self.setFixedSize(self.cols*CELL_SIZE, self.rows*CELL_SIZE)

        self.step = 0
        self.robot_dir = RIGHT  # initial orientation

        # Timer to move robot
        self.timer = QTimer()
        self.timer.timeout.connect(self.move_robot)
        self.timer.start(500)  # move every 500ms

    def move_robot(self, pose):
        self.current_pose = pose
        self.robot_dir = int(self.current_pose.z)  # Update direction from message
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Draw maze walls as lines
        painter.setPen(QPen(Qt.black, WALL_THICKNESS))
        for y in range(self.rows):
            for x in range(self.cols):
                if self.maze[y][x] == 1:
                    top_left_x = x * CELL_SIZE
                    top_left_y = y * CELL_SIZE
                    if y == 0 or self.maze[y-1][x] == 0:  # top wall
                        painter.drawLine(top_left_x, top_left_y, top_left_x + CELL_SIZE, top_left_y)
                    if x == 0 or self.maze[y][x-1] == 0:  # left wall
                        painter.drawLine(top_left_x, top_left_y, top_left_x, top_left_y + CELL_SIZE)
                    if y == self.rows-1 or self.maze[y+1][x] == 0:  # bottom wall
                        painter.drawLine(top_left_x, top_left_y + CELL_SIZE, top_left_x + CELL_SIZE, top_left_y + CELL_SIZE)
                    if x == self.cols-1 or self.maze[y][x+1] == 0:  # right wall
                        painter.drawLine(top_left_x + CELL_SIZE, top_left_y, top_left_x + CELL_SIZE, top_left_y + CELL_SIZE)

        # Draw robot
        rx, ry = self.current_pose.x, self.current_pose.y
        center_x = rx * CELL_SIZE + CELL_SIZE//2
        center_y = ry * CELL_SIZE + CELL_SIZE//2
        radius = CELL_SIZE//3

        # Draw body
        painter.setBrush(QBrush(QColor(0,0,255)))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(center_x - radius//2, center_y - radius//2, radius, radius)

        # Draw left and right hands depending on orientation
        painter.setPen(QPen(Qt.red, 3))
        offset = radius//2
        if self.robot_dir == UP:
            painter.drawLine(center_x - offset, center_y, center_x - offset, center_y - radius)  # left hand
            painter.drawLine(center_x + offset, center_y, center_x + offset, center_y - radius)  # right hand
        elif self.robot_dir == DOWN:
            painter.drawLine(center_x - offset, center_y, center_x - offset, center_y + radius)
            painter.drawLine(center_x + offset, center_y, center_x + offset, center_y + radius)
        elif self.robot_dir == LEFT:
            painter.drawLine(center_x, center_y - offset, center_x - radius, center_y - offset)
            painter.drawLine(center_x, center_y + offset, center_x - radius, center_y + offset)
        elif self.robot_dir == RIGHT:
            painter.drawLine(center_x, center_y - offset, center_x + radius, center_y - offset)
            painter.drawLine(center_x, center_y + offset, center_x + radius, center_y + offset)

class MazeWindow(QMainWindow):
    def __init__(self, maze):
        super().__init__()
        self.setWindowTitle("Maze Robot Viewer")
        self.maze_widget = MazeWidget(maze)
        self.setCentralWidget(self.maze_widget)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MazeWindow(maze)
    window.show()
    sys.exit(app.exec_())
