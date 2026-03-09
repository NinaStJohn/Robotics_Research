#pragma once
#include <vector>
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

        // note: const ref return, and spelling
        virtual const std::vector<Action>& actions() const = 0;

        // bool can_enter(const GridWorld& world, Pos pos) const;

    protected:
        Pos pos_;
        std::unordered_set<std::string> constraints_;

};


/*
 * Robot types:
 *
 * 
*/


class Turtlebot : public Robot {
    public:
        using Robot::Robot; // inherits Robot(Pos)

        const std::vector<Action>& actions() const override;

    private:
        static const std::vector<Action> kActions_;
};