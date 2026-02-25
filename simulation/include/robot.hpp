#pragma once
#include <vector>
#include <utility>

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

        // note: const ref return, and spelling
        virtual const std::vector<Action>& actions() const = 0;

    protected:
        Pos pos_;
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