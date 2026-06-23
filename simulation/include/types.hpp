#pragma once

#include <functional>
#include <utility>

struct Pos {
    int x;
    int y;
    bool operator==(const Pos& o) const { return x == o.x && y == o.y; }
};

struct PosNbaHash {
    std::size_t operator()(const std::pair<Pos, unsigned>& k) const {
        std::size_t h = std::hash<int>{}(k.first.x);
        h ^= std::hash<int>{}(k.first.y)  + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<unsigned>{}(k.second) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};