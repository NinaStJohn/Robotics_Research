#pragma once
#include <vector>
#include <utility>

class GridWorld{
    public:
        using Pos = std::pair<int,int>;
        GridWorld(int x, int h);

        int width() const;
        int height() const;

        bool in_bounds(int x, int y) const;
        void set_blocked(int x, int y, bool blocked = true);
        bool is_blocked(int x, int y) const;

        std::vector<Pos> neighbors4(int x, int y) const;

    private:
        int w_, h_;
        std::vector<std::vector<unsigned char>> blocked_;
};