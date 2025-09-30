#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/bt_factory.h"
#include "std_msgs/msg/int32.hpp"
#include <string>

enum Direction {UP, DOWN, LEFT, RIGHT};
enum CellType {WALL=1, PATH=0, START=2, GOAL=3};
struct Position {
    uint8_t x;
    uint8_t y;
    Direction direction;
};

Position initialPosition = {1, 1, RIGHT};

using Maze = std::vector<std::vector<int>>;

// function to get the next position based on current position and direction
Position getNextPosition(const Position current, Direction dir) {
    Position next = current;
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
*  @param current The current position of the robot
*  @param rel the direction of the frame in which you want to get relative direction to it
*  @return The absolute direction (relative to maze)
*/
Direction absoluteDirection(const Position current, Direction rel) {
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

void move(Position& current, Direction move_dir) {
    current = getNextPosition(current, move_dir);
    current.direction = move_dir;
}

Maze maze = {
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
        {1,2,0,0,0,1,0,0,0,1,0,0,0,3,1},
        {1,0,1,1,0,1,0,1,0,1,0,1,1,0,1},
        {1,0,1,0,0,0,0,1,0,0,0,0,1,0,1},
        {1,0,1,0,1,1,1,1,1,1,1,0,1,0,1},
        {1,0,0,0,1,0,0,0,0,0,1,0,1,0,1},
        {1,1,1,0,1,0,1,1,1,0,1,0,1,0,1},
        {1,0,0,0,0,0,1,0,0,0,1,0,0,0,1},
        {1,0,1,1,1,1,1,0,1,1,1,1,1,0,1},
        {1,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
    };

using namespace std::chrono_literals;

class IsWallHandWayOpen : public BT::StatefulActionNode{
public:
    IsWallHandWayOpen(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config){}

    BT::NodeStatus onStart() {
        currentPosition = config().blackboard->get<Position>("currentPosition");
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
        // next position toward the wall hand
        Position nextPosition = getNextPosition(currentPosition, absoluteDirection(currentPosition, wall_hand_));
        if(maze[nextPosition.y][nextPosition.x] == PATH) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Wall hand way is open");
            currentPosition = nextPosition;
            config().blackboard->set("currentPosition", currentPosition);
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
    Position currentPosition;
};

class IsGoal : public BT::StatefulActionNode {
public:
    IsGoal(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config) {        }
    
    BT::NodeStatus onStart() {
        currentPosition = config().blackboard->get<Position>("currentPosition");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() {
        if(maze[currentPosition.y][currentPosition.x] == GOAL) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal Reached at (x=%d, y=%d)", currentPosition.x, currentPosition.y);
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
    Position currentPosition;
};

class IsFrontOpen : public BT::StatefulActionNode{
public:
    IsFrontOpen(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config) {}

    BT::NodeStatus onStart() {
        currentPosition = config().blackboard->get<Position>("currentPosition");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() {
        Position nextPosition = getNextPosition(currentPosition, absoluteDirection(currentPosition, UP));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Checking front cell at (x=%d, y=%d)", nextPosition.x, nextPosition.y);
        if(maze[nextPosition.y][nextPosition.x] == PATH) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Front way is open");
            currentPosition = nextPosition;
            config().blackboard->set("currentPosition", currentPosition);
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
    Position currentPosition;
};

class IsOtherHandOpen : public BT::StatefulActionNode{
public:
    IsOtherHandOpen(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config) {}
    
    BT::NodeStatus onStart() {
        currentPosition = config().blackboard->get<Position>("currentPosition");
        other_hand_ = (config().blackboard->get<Direction>("wall_hand") == LEFT) ? RIGHT : LEFT;
        return BT::NodeStatus::RUNNING;
    } 

    BT::NodeStatus onRunning() {
        Position nextPosition = getNextPosition(currentPosition, absoluteDirection(currentPosition, other_hand_));
        if(maze[nextPosition.y][nextPosition.x] == PATH) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Other hand way is open");
            currentPosition = nextPosition;
            config().blackboard->set("currentPosition", currentPosition);
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
    Position currentPosition;
    Direction other_hand_;
};

class IsBackOpen : public BT::StatefulActionNode{
public:
    IsBackOpen(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config) {}

    BT::NodeStatus onStart() {
        currentPosition = config().blackboard->get<Position>("currentPosition");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() {
        Position nextPosition = getNextPosition(currentPosition, absoluteDirection(currentPosition, DOWN));
        if(maze[nextPosition.y][nextPosition.x] == PATH) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Back way is open");
            currentPosition = nextPosition;
            config().blackboard->set("currentPosition", currentPosition);
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
    Position currentPosition;
};

class HandMazeSolver : public rclcpp::Node {
public:
    HandMazeSolver() : Node("hand_maze_solver") {
        RCLCPP_INFO(this->get_logger(), "Starting One Hand Maze Solver Node");
        first_ = true;
        timer_ = this->create_wall_timer( 0.05s, std::bind(&HandMazeSolver::tick_function, this));
        blackboard_ = BT::Blackboard::create();
    }


private:
    void init_btree() {

        blackboard_->set<rclcpp::Node::SharedPtr>("node", this->shared_from_this());
        blackboard_->set<Position>("currentPosition", initialPosition);
        blackboard_->set<Maze>("maze", maze);
        blackboard_->set<Direction>("wall_hand", LEFT); // keep left hand on the wall
        //set tree
        factory_.registerNodeType<IsWallHandWayOpen>("IsWallHandWayOpen");
        factory_.registerNodeType<IsGoal>("IsGoal");
        factory_.registerNodeType<IsFrontOpen>("IsFrontOpen");
        factory_.registerNodeType<IsOtherHandOpen>("IsOtherHandOpen");
        factory_.registerNodeType<IsBackOpen>("IsBackOpen");
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
      auto currentPosition = blackboard_->get<Position>("currentPosition");
      RCLCPP_INFO(this->get_logger(), "CURRENT POSITION: x=%d, y=%d, direction=%d", currentPosition.x, currentPosition.y, currentPosition.direction);
      tree_.tickOnce();
    }
    BT::BehaviorTreeFactory factory_;
    rclcpp::TimerBase::SharedPtr timer_;
    BT::Blackboard::Ptr blackboard_;
    bool first_;
    BT::Tree tree_;
};

int main(int argc, char * argv[]) {

  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr BT_executor_node = std::make_shared<HandMazeSolver>();
  rclcpp::spin( BT_executor_node );
  rclcpp::shutdown();

  return 0;

}