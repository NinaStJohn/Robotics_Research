#include "grid_world.hpp"
#include <iostream>

GridWorld::GridWorld(int w, int h)
    : w_(w), h_(h), blocked_(h, std::vector<unsigned char>(w, 0)) {}

int GridWorld::width() const { return w_; }
int GridWorld::height() const { return h_; }

bool GridWorld::in_bounds(int x, int y) const {
    return 0 <= x && x < w_ && 0 <= y && y < h_;
}

void GridWorld::set_blocked(int x, int y, bool blocked) {
    if (!in_bounds(x,y)) return;
    blocked_[y][x] = blocked ? 1 : 0;
}

bool GridWorld::is_blocked(int x, int y) const {
    if (!in_bounds(x,y)) return true;
    return blocked_[y][x] != 0;
}

std::vector<GridWorld::Pos> GridWorld::neighbors4(int x, int y) const {
    std::vector<Pos> out;
    const int dx[4] = {-1, 1, 0, 0};
    const int dy[4] = { 0, 0,-1, 1};
    for (int i=0;i<4;i++){
        int nx = x + dx[i], ny = y + dy[i];
        if (in_bounds(nx,ny) && !is_blocked(nx,ny))
            out.push_back({nx,ny});
    }
    return out;
}

// -----------------------------------------------------
// Map labeling
// "label grid"
//
// -----------------------------------------------------

// static std::size_t idx(
//     int x, int y, int w
// ){
//     return static_cast<std::size_t>(y * w + x);
// }

void GridWorld::define_label(
    const std::string& name
){
    if (label_bit_.find(name) != label_bit_.end())
        return;

    int next = static_cast<int>(label_bit_.size());
    if (next >= 64) {
        std::cerr << "Too many labels (max 64)\n";
        return;
    }
    label_bit_[name] = next;

    if (labels_.empty()) {
        labels_.assign(static_cast<std::size_t>(w_ * h_), 0ULL);
    }
}

void GridWorld::set_label(const Pos& pnt,
                          const std::string& name,
                          bool value)
{
    int x = pnt.first;
    int y = pnt.second;

    if (!in_bounds(x, y)) return;

    // allocate bit if needed
    if (label_bit_.find(name) == label_bit_.end()) {
        int next = static_cast<int>(label_bit_.size());
        label_bit_[name] = next;
    }

    if (labels_.empty()) {
        labels_.assign(static_cast<std::size_t>(w_ * h_), 0ULL);
    }

    int bit = label_bit_[name];
    std::uint64_t mask = (1ULL << bit);
    std::size_t idx = static_cast<std::size_t>(y * w_ + x);

    if (value) labels_[idx] |= mask;
    else       labels_[idx] &= ~mask;
}

bool GridWorld::has_label(const Pos& pnt,
                          const std::string& name) const
{
    int x = pnt.first;
    int y = pnt.second;

    if (!in_bounds(x, y)) return false;

    std::unordered_map<std::string,int>::const_iterator it =
        label_bit_.find(name);

    if (it == label_bit_.end()) return false;
    if (labels_.empty()) return false;

    int bit = it->second;
    std::uint64_t mask = (1ULL << bit);
    std::size_t idx = static_cast<std::size_t>(y * w_ + x);

    return (labels_[idx] & mask) != 0ULL;
}