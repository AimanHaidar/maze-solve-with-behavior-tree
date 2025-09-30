#ifndef HELPERS_HPP
#define HELPERS_HPP

#include "maze_solver/types.hpp"
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

#endif