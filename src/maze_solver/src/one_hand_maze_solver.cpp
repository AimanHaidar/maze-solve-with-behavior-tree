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

class IsWallHandOpen : public BT::StatefulActionNode{
public:
    IsWallHandOpen(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config) {
            current_position = config.blackboard->get<Position>("current_position");
        }

    BT::NodeStatus onStart() override {
        std::string wall_hand_;
        getInput<std::string>("wall_hand", wall_hand_);
        if (wall_hand_=="left") {
            wall_hand_ = LEFT;
        } else if (wall_hand_=="right") {
            wall_hand_ = RIGHT;
        } else {
            throw std::runtime_error("Invalid wall_hand input. Use 'left' or 'right'.");
        }

        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onRunning(){
        // next position toward the wall hand
        Position nextPosition = getNextPosition(current_position, relativeDirection(current_position, wall_hand_));
        if(maze[nextPosition.y][nextPosition.x] == WALL) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Wall hand %s is open");
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Wall hand %s is closed");
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    Direction wall_hand_;
    Position current_position;
};

class HandMazeSolver : public rclcpp::Node {
public:
    HandMazeSolver() : Node("hand_maze_solver") {
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
        this->declare_parameter<std::string>("tree_xml_file", "");
        std::string tree_file;
        this->get_parameter("tree_xml_file", tree_file);
        tree_ = factory_.createTreeFromFile(tree_file, blackboard_);
        
    }

    void tick_function() {
      if( first_) {
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