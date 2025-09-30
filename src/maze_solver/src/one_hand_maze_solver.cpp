#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/bt_factory.h"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <string>
#include <iostream>
#include "maze_solver/mazes.hpp"
#include "maze_interfaces/srv/maze.hpp"
#include "maze_solver/types.hpp"
#include "maze_solver/nodes/maze_solver_nodes.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace hand_maze_solver;

class HandMazeSolver : public rclcpp::Node {
public:
    HandMazeSolver() : Node("hand_maze_solver") {
        RCLCPP_INFO(this->get_logger(), "Starting One Hand Maze Solver Node");
        first_ = true;
        start_ = false;
        timer_ = this->create_wall_timer( 0.05s, std::bind(&HandMazeSolver::tick_function, this));
        blackboard_ = BT::Blackboard::create();
        maze_service_ = this->create_service<maze_interfaces::srv::Maze>(
            "maze", std::bind(&HandMazeSolver::callbackMaze, this, _1, _2));
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
        factory_.registerNodeType<PublishToGui>("PublishToGui");
        // Load XML file
        this->declare_parameter<std::string>("tree_xml_file", "/home/aymanhadair/ROS2/BahaviorTree/src/maze_solver/trees/one_hand_maze_solver.xml");
        std::string tree_file;
        this->get_parameter("tree_xml_file", tree_file);
        tree_ = factory_.createTreeFromFile(tree_file, blackboard_);
        
    }

    void tick_function() {
        if(start_){
            if( first_) {
                RCLCPP_INFO(this->get_logger(), "Initializing Behavior Tree");
                init_btree();
                first_ = false;
            }
            auto currentPose = blackboard_->get<Pose>("currentPose");
            RCLCPP_INFO(this->get_logger(), "CURRENT Pose: x=%d, y=%d, direction=%d", currentPose.x, currentPose.y, currentPose.direction);
            tree_.tickOnce();
        }
    }

    void callbackMaze(
        const std::shared_ptr<maze_interfaces::srv::Maze::Request> request,
        std::shared_ptr<maze_interfaces::srv::Maze::Response> response)
        {
        start_ = true;
        RCLCPP_INFO(this->get_logger(), "Received maze size request: %s", request->name.c_str());
        if(request->name == "small") {
            maze = small_maze;
        } else if(request->name == "middle") {
            maze = middle_maze;
        } else if(request->name == "big") {
            maze = big_maze;
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid maze size %s request. Defaulting to 'small'.", request->name.c_str());
            maze = small_maze;
        }
        response->rows = maze.size();
        response->cols = maze[0].size();

        for (int y = 0; y < maze.size(); ++y) {
            for (int x = 0; x < maze[0].size(); ++x) {
                response->maze.push_back(static_cast<double>(maze[y][x]));
            }
        }
    }
    BT::BehaviorTreeFactory factory_;
    rclcpp::TimerBase::SharedPtr timer_;
    BT::Blackboard::Ptr blackboard_;
    bool first_;
    bool start_;
    BT::Tree tree_;
    Maze maze;
    rclcpp::Service<maze_interfaces::srv::Maze>::SharedPtr maze_service_;
};

int main(int argc, char * argv[]) {

  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr BT_executor_node = std::make_shared<HandMazeSolver>();
  rclcpp::spin( BT_executor_node );
  rclcpp::shutdown();

  return 0;

}