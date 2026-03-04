#pragma once
#include <vector>
#include <utility>
#include "grid_world.hpp"

class Robot {
    public:
        using Pos = std::pair<int,int>;

        enum class Action {
            Stay,
            Left,
            Right,
            Up,
            Down
            // Later: Open, Close, Pickup, Drop...
        };

        Robot(int x, int y) : pos_{x,y} {}
        virtual ~Robot() = default;

        Pos position() const;
        void set_position(int x, int y);

        void set_constraints(const std::string& ap);
        void clear_constraints();

        // note: const ref return, and spelling
        virtual const std::vector<Action>& actions() const = 0;

        bool can_enter(const GridWorld& word, Pos pos);

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
        using Robot::Robot; // inherits Robot(int,int)

        const std::vector<Action>& actions() const override;

    private:
        static const std::vector<Action> kActions_;
};