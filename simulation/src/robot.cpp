/*
    Robot cabalities
    - use this file to create NBA
    - should just define actions that the 'robot' 
    can take
    - hopefully can be inflatted to add more features
*/

#include "robot.hpp"

// public returns
Robot::Pos Robot::position() const {
    return pos_;
}

double Robot::action_cost(Action a) const {
    std::unordered_map<Action, double>::const_iterator it = action_costs_.find(a);
    return (it != action_costs_.end()) ? it->second : 1.0;
}

void Robot::set_action_cost(Action a, double cost) {
    action_costs_[a] = cost;
}

void Robot::set_position(Pos pos) {
    pos_ = pos;
}

// set limits for ability to access or not access
// right now assume that you can access everything
// make make restricted areas where access is NOT assumed? 
void Robot::set_constraints(
    const std::string& ap
){
    constraints_.insert(ap);
}

void Robot::clear_constraints(
    void
){
    constraints_.clear();
}

// // temporary simple policy:
// bool Robot::can_enter(const GridWorld& world, Pos pos) const {
//     (void)world;
//     (void)pos;
//     return true;
// }

// defining the turtlebot action set (static storage)

const std::vector<Robot::Action> Turtlebot::kActions_ = {
    Robot::Action::Stay,
    Robot::Action::Left,
    Robot::Action::Right,
    Robot::Action::Up,
    Robot::Action::Down
};

const std::vector<Robot::Action>& Turtlebot::actions() const {
    return kActions_;
}