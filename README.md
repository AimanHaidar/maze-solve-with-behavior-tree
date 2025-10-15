# Behavior Tree Maze Solver With ROS2

these packages contain example of using behavior tree to solve maze by sticking one hand with the wall and show graphical pyqt5 window for the search process

more about behavior Trees at [BehaviorTreeCPP Tutorials](https://www.behaviortree.dev/docs/category/basic-concepts)

---

## Screenshots
**maze:**
![Alt text](screenshots/maze.png)

---

**behavior tree:**
the "Is**" Nodes check and move the robot

<img src="screenshots/behavior_tree.svg" alt="Behavior Tree" height="400" width="500"/>


<!-- ![Alt text]() -->


## Usage
```bash
cd maze-solve-with-behavior-tree
uv sync

colcon build
source install/setup.bash
# launch with maze_size small,big,middle
ros2 launch launcher start.launch.py maze_size:=big
```
