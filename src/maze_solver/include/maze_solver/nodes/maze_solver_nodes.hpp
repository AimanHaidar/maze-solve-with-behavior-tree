#ifndef MAZE_SOLVER_NODES_HPP
#define MAZE_SOLVER_NODES_HPP

#include "rclcpp/rclcpp.hpp"
#include "maze_solver/types.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/bt_factory.h"
#include "maze_solver/utils/helpers.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace hand_maze_solver{
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
        if(maze[nextPose.y][nextPose.x] != WALL) {
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
        if(maze[nextPose.y][nextPose.x] != WALL) {
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
        if(maze[nextPose.y][nextPose.x] != WALL) {
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
        if(maze[nextPose.y][nextPose.x] != WALL) {
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

class PublishToGui : public BT::StatefulActionNode {
public:
    PublishToGui(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config) {}

    BT::NodeStatus onStart(){
        std::string topic = getInput<std::string>("topic_name").value();
        publisher_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node")->create_publisher<geometry_msgs::msg::Point>(topic, 10);
        currentPose = config().blackboard->get<Pose>("currentPose");
        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus onRunning(){
        geometry_msgs::msg::Point msg;
        msg.x = currentPose.x;
        msg.y = currentPose.y;
        msg.z = currentPose.direction;
        publisher_->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }
    void onHalted() { return; }
    static BT::PortsList providedPorts() {
      return {
        BT::InputPort<std::string>("topic_name"),
      };
    }
private:
    Pose currentPose;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
};

}

#endif