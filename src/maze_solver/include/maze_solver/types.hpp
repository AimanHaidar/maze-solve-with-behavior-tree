#ifndef TYPES_HPP
#define TYPES_HPP

#include <stdint.h>
#include <vector>
#include "maze_solver/types.hpp"

enum Direction {UP, DOWN, LEFT, RIGHT};
enum CellType {WALL=1, PATH=0, START=2, GOAL=3};
struct Pose {
    uint8_t x;
    uint8_t y;
    Direction direction;
    bool operator==(const Pose& other) const {
        return x == other.x && y == other.y && direction == other.direction;
    }

    // Optional constructor for convenience
    Pose() : x(0), y(0), direction(UP) {}
    Pose(int x_, int y_, Direction dir_) : x(x_), y(y_), direction(dir_) {}
};

using Maze = std::vector<std::vector<int>>;

using MazeRecord = std::vector<std::vector<Pose>>;

#endif