#pragma once
#include <vector>
#include <utility>
#include <unordered_map>
#include <cstdint>
#include <string>

class GridWorld {
    public:
        using Pos = std::pair<int,int>;

        GridWorld(int w, int h);

        int width() const;
        int height() const;

        bool in_bounds(int x, int y) const;
        void set_blocked(int x, int y, bool blocked = true);
        bool is_blocked(int x, int y) const;

        std::vector<Pos> neighbors4(int x, int y) const;

        // label thins
        void define_label(const std::string& name);
        void set_label(const Pos& pnt, const std::string& name, bool value);
        bool has_label(const Pos& pnt, const std::string& name) const;

        // may need for label
        std::uint64_t label_mask(int x, int y) const;

    private:
        int w_, h_;
        std::vector<std::vector<unsigned char>> blocked_;

        // Label map
        std::unordered_map<std::string, int> label_bit_;         // name -> bit index
        std::vector<std::uint64_t> labels_;                      // size w*h, each cell mask

};