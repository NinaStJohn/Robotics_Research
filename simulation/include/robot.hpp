#pragma once
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <string>
#include "types.hpp"

class Robot {
    public:

        enum class Action {
            Stay,
            Left,
            Right,
            Up,
            Down
            // Later: Open, Close, Pickup, Drop...
        };
        using Pos = ::Pos;

        // init like dis
        Robot(Pos pos) : pos_(pos) {}
        virtual ~Robot() = default;

        Pos position() const;
        void set_position(Pos pos);

        void set_constraints(const std::string& ap);
        void clear_constraints();

        double action_cost(Action a) const;
        void   set_action_cost(Action a, double cost);

        // note: const ref return, and spelling
        virtual const std::vector<Action>& actions() const = 0;

        // bool can_enter(const GridWorld& world, Pos pos) const;

    protected:
        Pos pos_;
        std::unordered_set<std::string> constraints_;
        std::unordered_map<Action, double> action_costs_;

};


/*
 * Robot types:
 *
 * 
*/


class Turtlebot : public Robot {
    public:
        explicit Turtlebot(Pos pos) : Robot(pos) {
            action_costs_[Action::Stay]  = 0.0;
            action_costs_[Action::Left]  = 1.0;
            action_costs_[Action::Right] = 1.0;
            action_costs_[Action::Up]    = 1.0;
            action_costs_[Action::Down]  = 1.0;
        }

        const std::vector<Action>& actions() const override;

    private:
        static const std::vector<Action> kActions_;
};