import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget
from PyQt5.QtGui import QPainter, QColor, QPen
from PyQt5.QtCore import Qt

# Example maze
# 1 = wall, 0 = free
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

CELL_SIZE = 40  # pixels
WALL_THICKNESS = 4

class MazeWidget(QWidget):
    def __init__(self, maze):
        super().__init__()
        self.maze = maze
        self.rows = len(maze)
        self.cols = len(maze[0])
        self.setFixedSize(self.cols*CELL_SIZE, self.rows*CELL_SIZE)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setPen(QPen(Qt.black, WALL_THICKNESS))

        for y in range(self.rows):
            for x in range(self.cols):
                if self.maze[y][x] == 1:
                    # Draw the walls as lines around the cell
                    top_left_x = x * CELL_SIZE
                    top_left_y = y * CELL_SIZE

                    # Draw top wall
                    if y == 0 or self.maze[y-1][x] == 0:
                        painter.drawLine(top_left_x, top_left_y, top_left_x + CELL_SIZE, top_left_y)
                    # Draw left wall
                    if x == 0 or self.maze[y][x-1] == 0:
                        painter.drawLine(top_left_x, top_left_y, top_left_x, top_left_y + CELL_SIZE)
                    # Draw bottom wall
                    if y == self.rows-1 or self.maze[y+1][x] == 0:
                        painter.drawLine(top_left_x, top_left_y + CELL_SIZE, top_left_x + CELL_SIZE, top_left_y + CELL_SIZE)
                    # Draw right wall
                    if x == self.cols-1 or self.maze[y][x+1] == 0:
                        painter.drawLine(top_left_x + CELL_SIZE, top_left_y, top_left_x + CELL_SIZE, top_left_y + CELL_SIZE)

class MazeWindow(QMainWindow):
    def __init__(self, maze):
        super().__init__()
        self.setWindowTitle("Maze Viewer")
        self.setCentralWidget(MazeWidget(maze))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MazeWindow(maze)
    window.show()
    sys.exit(app.exec_())
