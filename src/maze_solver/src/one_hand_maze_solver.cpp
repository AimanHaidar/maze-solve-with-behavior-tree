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

Position initial_position = {1, 1, RIGHT};

using Maze = std::vector<std::vector<int>>;

// function to get the next position based on current position and direction
Position getNextPosition(const Position& current, Direction dir) {
    Position next = current;
    switch (dir) {
        case UP:    next.y -= 1; break;
        case DOWN:  next.y += 1; break;
        case LEFT:  next.x -= 1; break;
        case RIGHT: next.x += 1; break;
    }
    return next;
}

Direction relativeDirection(const Position& current, Direction rel) {
    switch (current.direction) {
        case UP:
            if (rel == LEFT) return LEFT;
            if (rel == RIGHT) return RIGHT;
            break;
        case DOWN:
            if (rel == LEFT) return RIGHT;
            if (rel == RIGHT) return LEFT;
            break;
        case LEFT:
            if (rel == LEFT) return DOWN;
            if (rel == RIGHT) return UP;
            break;
        case RIGHT:
            if (rel == LEFT) return UP;
            if (rel == RIGHT) return DOWN;
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
        : BT::StatefulActionNode(name, config) {
            this->config = config;
        }

    BT::NodeStatus onStart() {
        current_position = config.blackboard->get<Position>("current_position");
        std::string wall_hand_str;
        getInput<std::string>("wall_hand", wall_hand_str);
        if (wall_hand_str == "left") {
            this->wall_hand_ = LEFT;
        } else if (wall_hand_str == "right") {
            this->wall_hand_ = RIGHT;
        } else {
            throw std::runtime_error("Invalid wall_hand input. Use 'left' or 'right'.");
        }

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning(){
        // next position toward the wall hand
        Position nextPosition = getNextPosition(current_position, relativeDirection(current_position, wall_hand_));
        if(maze[nextPosition.y][nextPosition.x] == PATH) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Wall hand way is open");
            move(current_position, relativeDirection(current_position, wall_hand_));
            config.blackboard->set("current_position", current_position);
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
    Position current_position;
    BT::NodeConfiguration config;
};

class HandMazeSolver : public rclcpp::Node {
public:
    HandMazeSolver() : Node("hand_maze_solver") {
        RCLCPP_INFO(this->get_logger(), "Starting One Hand Maze Solver Node");
        first_ = true;
        timer_ = this->create_wall_timer( 0.5s, std::bind(&HandMazeSolver::tick_function, this));
        blackboard_ = BT::Blackboard::create();
    }


private:
    void init_btree() {

        blackboard_->set<rclcpp::Node::SharedPtr>("node", this->shared_from_this());
        blackboard_->set<Position>("current_position", initial_position);
        blackboard_->set<Maze>("maze", maze);
        // TODO  set tree
        factory_.registerNodeType<IsWallHandWayOpen>("IsWallHandWayOpen");
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