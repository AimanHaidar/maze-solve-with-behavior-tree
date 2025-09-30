#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/bt_factory.h"
#include "std_msgs/msg/int32.hpp"
#include <string>
#include <iostream>
#include "maze_solver/mazes.hpp"

// function to get the next Pose based on current Pose and direction
Pose getNextPose(const Pose current, Direction dir) {
    Pose next = current;
    switch (dir) {
        case UP:    next.y -= 1; break;
        case DOWN:  next.y += 1; break;
        case LEFT:  next.x -= 1; break;
        case RIGHT: next.x += 1; break;
    }
    if(dir!=current.direction){
        next.direction = dir;
    }
    return next;
}
/**
*  @param current The current Pose of the robot
*  @param rel the direction of the frame in which you want to get relative direction to it
*  @return The absolute direction (relative to maze)
*/
Direction absoluteDirection(const Pose current, Direction rel) {
    switch (current.direction) {
        case UP:
            if (rel == LEFT) return LEFT;
            else if (rel == RIGHT) return RIGHT;
            else if (rel == UP) return UP;
            else if (rel == DOWN) return DOWN;
            break;
        case DOWN:
            if (rel == LEFT) return RIGHT;
            else if (rel == RIGHT) return LEFT;
            else if (rel == UP) return DOWN;
            else if (rel == DOWN) return UP;
            break;
        case LEFT:
            if (rel == LEFT) return DOWN;
            else if (rel == RIGHT) return UP;
            else if (rel == UP) return LEFT;
            else if (rel == DOWN) return RIGHT;
            break;
        case RIGHT:
            if (rel == LEFT) return UP;
            else if (rel == RIGHT) return DOWN;
            else if (rel == UP) return RIGHT;
            else if (rel == DOWN) return LEFT;
            break;
    }
}

void move(Pose& current, Direction move_dir) {
    current = getNextPose(current, move_dir);
    current.direction = move_dir;
}



using namespace std::chrono_literals;

class IsWallHandWayOpen : public BT::StatefulActionNode{
public:
    IsWallHandWayOpen(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config){}

    BT::NodeStatus onStart() {
        maze = config().blackboard->get<Maze>("maze");
        currentPose = config().blackboard->get<Pose>("currentPose");
        std::string wall_hand_str;
        getInput<std::string>("wall_hand", wall_hand_str);
        
        if (wall_hand_str == "left") {
            this->wall_hand_ = LEFT;
        } else if (wall_hand_str == "right") {
            this->wall_hand_ = RIGHT;
        } else {
            throw std::runtime_error("Invalid wall_hand input. Use 'left' or 'right'.");
        }
        config().blackboard->set("wall_hand", this->wall_hand_);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning(){
        // next Pose toward the wall hand
        Pose nextPose = getNextPose(currentPose, absoluteDirection(currentPose, wall_hand_));
        if(maze[nextPose.y][nextPose.x] == PATH || maze[nextPose.y][nextPose.x] == GOAL) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Wall hand way is open");
            currentPose = nextPose;
            config().blackboard->set("currentPose", currentPose);
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Wall hand is closed");
            return BT::NodeStatus::FAILURE;
        }
    }

    void onHalted() { return; }

    static BT::PortsList providedPorts() {
      return {
        BT::InputPort<std::string>("wall_hand"),
      };
    }

private:
    Direction wall_hand_;
    Pose currentPose;
    Maze maze;
};

class IsGoal : public BT::StatefulActionNode {
public:
    IsGoal(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config) {        }
    
    BT::NodeStatus onStart() {
        maze = config().blackboard->get<Maze>("maze");
        currentPose = config().blackboard->get<Pose>("currentPose");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() {
        if(maze[currentPose.y][currentPose.x] == GOAL) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal Reached at (x=%d, y=%d)", currentPose.x, currentPose.y);
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Not at Goal yet.");
            return BT::NodeStatus::FAILURE;
        }
    }
    void onHalted() { return; }
    static BT::PortsList providedPorts() {
      return {};
    }

private:
    Pose currentPose;
    Maze maze;
};

class IsFrontOpen : public BT::StatefulActionNode{
public:
    IsFrontOpen(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config) {}

    BT::NodeStatus onStart() {
        maze = config().blackboard->get<Maze>("maze");
        currentPose = config().blackboard->get<Pose>("currentPose");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() {
        Pose nextPose = getNextPose(currentPose, absoluteDirection(currentPose, UP));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Checking front cell at (x=%d, y=%d)", nextPose.x, nextPose.y);
        if(maze[nextPose.y][nextPose.x] == PATH || maze[nextPose.y][nextPose.x] == GOAL) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Front way is open");
            currentPose = nextPose;
            config().blackboard->set("currentPose", currentPose);
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Front way is closed");
            return BT::NodeStatus::FAILURE;
        }
    }

    void onHalted() { return; }

    static BT::PortsList providedPorts() {
      return {};
    }

private:
    Maze maze;
    Pose currentPose;
};

class IsOtherHandOpen : public BT::StatefulActionNode{
public:
    IsOtherHandOpen(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config) {}
    
    BT::NodeStatus onStart() {
        maze = config().blackboard->get<Maze>("maze");
        currentPose = config().blackboard->get<Pose>("currentPose");
        other_hand_ = (config().blackboard->get<Direction>("wall_hand") == LEFT) ? RIGHT : LEFT;
        return BT::NodeStatus::RUNNING;
    } 

    BT::NodeStatus onRunning() {
        Pose nextPose = getNextPose(currentPose, absoluteDirection(currentPose, other_hand_));
        if(maze[nextPose.y][nextPose.x] == PATH || maze[nextPose.y][nextPose.x] == GOAL) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Other hand way is open");
            currentPose = nextPose;
            config().blackboard->set("currentPose", currentPose);
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Other hand way is closed");
            return BT::NodeStatus::FAILURE;
        }
    }

    void onHalted() { return; }
    static BT::PortsList providedPorts() {
      return {};
    }
private:
    Pose currentPose;
    Direction other_hand_;
    Maze maze;
};

class IsBackOpen : public BT::StatefulActionNode{
public:
    IsBackOpen(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config) {}

    BT::NodeStatus onStart() {
        maze = config().blackboard->get<Maze>("maze");
        currentPose = config().blackboard->get<Pose>("currentPose");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() {
        Pose nextPose = getNextPose(currentPose, absoluteDirection(currentPose, DOWN));
        if(maze[nextPose.y][nextPose.x] == PATH || maze[nextPose.y][nextPose.x] == GOAL) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Back way is open");
            currentPose = nextPose;
            config().blackboard->set("currentPose", currentPose);
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Back way is closed");
            return BT::NodeStatus::FAILURE;
        }
    }

    void onHalted() { return; }
    static BT::PortsList providedPorts() {
      return {};
    }
private:
    Pose currentPose;
    Maze maze;
};

class RecordPose : public BT::StatefulActionNode {
public:
    RecordPose(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config) {}

    BT::NodeStatus onStart(){
        currentPose = config().blackboard->get<Pose>("currentPose");
        recordedMaze = config().blackboard->get<MazeRecord>("recorded_maze");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning(){
        recordedMaze[currentPose.y][currentPose.x] = currentPose;
        config().blackboard->set("recorded_maze", recordedMaze);
        return BT::NodeStatus::SUCCESS;
    }
    void onHalted() { return; }

    static BT::PortsList providedPorts() {
      return {};
    }
private:
    Pose currentPose;
    MazeRecord recordedMaze;
};

class IsLoopState : public BT::ConditionNode {
public:
    IsLoopState(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}
    
    BT::NodeStatus tick() {
        currentPose = config().blackboard->get<Pose>("currentPose");
        recordedMaze = config().blackboard->get<MazeRecord>("recorded_maze");
        if((recordedMaze[currentPose.y][currentPose.x] == currentPose)) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "loop at (x=%d, y=%d)", currentPose.x, currentPose.y);
            return BT::NodeStatus::SUCCESS;
        } else{
            return BT::NodeStatus::FAILURE;
        }
    }
    void onHalted() { return; }
    static BT::PortsList providedPorts() {
      return {};
    }
private:
    MazeRecord recordedMaze;
    Pose currentPose;
};

class HandMazeSolver : public rclcpp::Node {
public:
    HandMazeSolver() : Node("hand_maze_solver") {
        RCLCPP_INFO(this->get_logger(), "Starting One Hand Maze Solver Node");
        first_ = true;
        timer_ = this->create_wall_timer( 0.05s, std::bind(&HandMazeSolver::tick_function, this));
        blackboard_ = BT::Blackboard::create();
        this->declare_parameter<std::string>("maze_size", "small");
        std::string maze_size;
        this->get_parameter("maze_size", maze_size);
        std::cout << "Selected maze size: " << maze_size << std::endl;
        if(maze_size == "small") {
            maze = small_maze;
        } else if(maze_size == "middle") {
            maze = middle_maze;
        } else if(maze_size == "big") {
            maze = big_maze;
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid maze_size %s parameter. Defaulting to 'small'.", maze_size.c_str());
            maze = small_maze;
        }

    }


private:
    void init_btree() {

        blackboard_->set<rclcpp::Node::SharedPtr>("node", this->shared_from_this());
        blackboard_->set<Pose>("currentPose", initialPose);
        blackboard_->set<Maze>("maze", maze);
        blackboard_->set<Direction>("wall_hand", LEFT); // keep left hand on the wall
        MazeRecord recorded_maze(maze.size(), std::vector<Pose>(maze[0].size(), Pose(-1,-1,UP)));
        blackboard_->set<MazeRecord>("recorded_maze", recorded_maze);
        //set tree
        factory_.registerNodeType<IsWallHandWayOpen>("IsWallHandWayOpen");
        factory_.registerNodeType<IsGoal>("IsGoal");
        factory_.registerNodeType<IsFrontOpen>("IsFrontOpen");
        factory_.registerNodeType<IsOtherHandOpen>("IsOtherHandOpen");
        factory_.registerNodeType<IsBackOpen>("IsBackOpen");
        factory_.registerNodeType<RecordPose>("RecordPose");
        factory_.registerNodeType<IsLoopState>("IsLoopState");
        // Load XML file
        this->declare_parameter<std::string>("tree_xml_file", "/home/aymanhadair/ROS2/BahaviorTree/src/maze_solver/trees/one_hand_maze_solver.xml");
        std::string tree_file;
        this->get_parameter("tree_xml_file", tree_file);
        tree_ = factory_.createTreeFromFile(tree_file, blackboard_);
        
    }

    void tick_function() {
      if( first_) {
        RCLCPP_INFO(this->get_logger(), "Initializing Behavior Tree");
          init_btree();
          first_ = false;
      }
      auto currentPose = blackboard_->get<Pose>("currentPose");
      RCLCPP_INFO(this->get_logger(), "CURRENT Pose: x=%d, y=%d, direction=%d", currentPose.x, currentPose.y, currentPose.direction);
      tree_.tickOnce();
    }
    BT::BehaviorTreeFactory factory_;
    rclcpp::TimerBase::SharedPtr timer_;
    BT::Blackboard::Ptr blackboard_;
    bool first_;
    BT::Tree tree_;
    Maze maze;
};

int main(int argc, char * argv[]) {

  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr BT_executor_node = std::make_shared<HandMazeSolver>();
  rclcpp::spin( BT_executor_node );
  rclcpp::shutdown();

  return 0;

}