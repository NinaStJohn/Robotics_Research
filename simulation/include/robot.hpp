# pragma once
#include <vector>
#include <utility>


class Robot{
    public:
        using Pos = std::pair<int,int>; // bot loc
        Robot(int x, int y);

        enum class Action {
            Stay,
            Left,
            Right,
            Up,
            Down
            /*
            Later:
            Open, Close, Pickup, Drop...
            
            */
        }

        Robot(int x, int y) : pos_{x,y} {}
        virtual ~Robot() = default;         // allows dynamic dispatch

        Pos position();
        void set_position(int, int);

        virtual cons std::vector<Action> & actions() const = 0;

    protected:
        Pos pos_; 

}

// Robot classes, using inheritance
class Turtlebot: public Robot{
    public:
        using Robot::Robot;
        const std::vector<Action>^& actions() const override;

    private:
        static const std::vector<Action> kActions_;
}