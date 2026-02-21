/*
    Robot cabalities
    - use this file to create NBA
    - should just define actions that the 'robot' 
    can take
    - hopefully can be inflatted to add more features
*/

#include "robot.hpp"

// public returns
Pos position() const {
    return pos_; 
}
void set_position(int x, int y) { 
    pos_ = {x,y};
}

// defining the turtlebot action set (static storage)

const std::vector<Robot::Action> Turtlebot::kActions_ = {
    Robot::Action::Stay,
    Robot::Action::Left,
    Robot::Action::Right,
    Robot::Action::Up,
    Robot::Action::Down
}

const std::vector<Robot::Actions>& Turtlebot::actions{} const {
    return kActions_;
}