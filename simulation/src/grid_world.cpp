#include "grid_world.hpp"
#include <iostream>

GridWorld::GridWorld(int w, int h)
    : w_(w), h_(h), blocked_(h, std::vector<unsigned char>(w, 0)) {}

int GridWorld::width() const { return w_; }
int GridWorld::height() const { return h_; }

bool GridWorld::in_bounds(Pos pos) const {
    return 0 <= pos.x && pos.x < w_ && 0 <= pos.y && pos.y < h_;
}

// blocked?
void GridWorld::set_blocked(Pos pos, bool blocked) {
    if (!in_bounds(pos)) return;
    blocked_[pos.y][pos.x] = blocked ? 1 : 0;
}

bool GridWorld::is_blocked(Pos pos) const {
    if (!in_bounds(pos)) return true;
    return blocked_[pos.y][pos.x] != 0;
}

// neigh 4
std::vector<Pos> GridWorld::neighbors4(Pos pos) const {
    std::vector<Pos> out;
    const int dx[4] = {-1, 1, 0, 0};
    const int dy[4] = { 0, 0,-1, 1};

    for (int i = 0; i < 4; i++) {
        Pos np{ pos.x + dx[i], pos.y + dy[i] };
        if (in_bounds(np) && !is_blocked(np))
            out.push_back(np);
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

void GridWorld::define_label(const std::string& name) {
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
    if (!in_bounds(pnt)) return;

    int x = pnt.x;
    int y = pnt.y;

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
    std::size_t cell = static_cast<std::size_t>(y * w_ + x);

    if (value) labels_[cell] |= mask;
    else       labels_[cell] &= ~mask;
}

bool GridWorld::has_label(const Pos& pnt,
                          const std::string& name) const
{
    if (!in_bounds(pnt)) return false;

    int x = pnt.x;
    int y = pnt.y;

    auto it = label_bit_.find(name);
    if (it == label_bit_.end()) return false;
    if (labels_.empty()) return false;

    int bit = it->second;
    std::uint64_t mask = (1ULL << bit);
    std::size_t cell = static_cast<std::size_t>(y * w_ + x);

    return (labels_[cell] & mask) != 0ULL;
}

const std::unordered_map<std::string,int>& GridWorld::label_map() const {
    return label_bit_;
}

std::vector<std::string> GridWorld::label_names() const {
    std::vector<std::string> out;
    out.reserve(label_bit_.size());
    for (const auto& kv : label_bit_) out.push_back(kv.first);
    return out;
}