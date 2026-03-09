#pragma once
#include <vector>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <string>
#include "types.hpp"

class GridWorld {
    public:
        GridWorld(int w, int h);

        int width() const;
        int height() const;

        // occupancy grid
        bool in_bounds(Pos pos) const;
        void set_blocked(Pos pos, bool blocked = true);
        bool is_blocked(Pos pos) const;
        // for vis get layer
        const std::vector<std::vector<unsigned char>>& blocked() const;

        // neighbors
        std::vector<Pos> neighbors4(Pos pos) const;

        // Label grid
        void define_label(const std::string& name);
        void set_label(const Pos& pnt, const std::string& name, bool value);
        bool has_label(const Pos& pnt, const std::string& name) const;
        const std::unordered_map<std::string,int>& label_map() const;
        std::vector<std::string> label_names() const;
        // for vis get layer
        const std::vector<std::uint64_t>& labels() const;

        // may need for label, per-cell mask
        std::uint64_t label_mask(int x, int y) const;

        // make the pos thing consistant
        // bool in_bounds(Pos p) const;
        // void set_blocked(Pos p, bool blocked=true);
        // bool is_blocked(Pos p) const;
        // std::uint64_t label_mask(Pos p) const;


    private:
        int w_, h_;
        std::vector<std::vector<unsigned char>> blocked_;

        // Label map
        std::unordered_map<std::string, int> label_bit_;         // name -> bit index
        std::vector<std::uint64_t> labels_;                      // size w*h, each cell mask

};