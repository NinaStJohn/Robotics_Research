#include "grid_world.hpp"
#include <iostream>

GridWorld::GridWorld(int w, int h)
    : w_(w), h_(h),
      occ_(h, std::vector<Occupancy>(w, Occupancy::Free)),
      discovered_(h, std::vector<unsigned char>(w, 0)) {}

int GridWorld::width() const { return w_; }
int GridWorld::height() const { return h_; }

bool GridWorld::in_bounds(Pos pos) const {
    return 0 <= pos.x && pos.x < w_ && 0 <= pos.y && pos.y < h_;
}

// occupancy
bool GridWorld::is_blocked(Pos pos) const {
    if (!in_bounds(pos)) return true;
    return occ_[pos.y][pos.x] != Occupancy::Free;
}

bool GridWorld::is_static(Pos pos) const {
    if (!in_bounds(pos)) return false;
    return occ_[pos.y][pos.x] == Occupancy::Static;
}

bool GridWorld::is_dynamic(Pos pos) const {
    if (!in_bounds(pos)) return false;
    return occ_[pos.y][pos.x] == Occupancy::Dynamic;
}

void GridWorld::set_static(Pos pos, bool value) {
    if (!in_bounds(pos)) return;
    Occupancy& cell = occ_[pos.y][pos.x];
    if (value) {
        if (cell == Occupancy::Dynamic) {
            std::cerr << "GridWorld::set_static(): refusing to overwrite Dynamic cell at ("
                      << pos.x << "," << pos.y << ")\n";
            return;
        }
        cell = Occupancy::Static;
    } else if (cell == Occupancy::Static) {
        cell = Occupancy::Free;
    }
}

void GridWorld::set_dynamic(Pos pos, bool value) {
    if (!in_bounds(pos)) return;
    Occupancy& cell = occ_[pos.y][pos.x];
    if (value) {
        if (cell == Occupancy::Static) {
            std::cerr << "GridWorld::set_dynamic(): refusing to overwrite Static cell at ("
                      << pos.x << "," << pos.y << ")\n";
            return;
        }
        cell = Occupancy::Dynamic;
    } else if (cell == Occupancy::Dynamic) {
        cell = Occupancy::Free;
    }
}

void GridWorld::set_free(Pos pos) {
    if (!in_bounds(pos)) return;
    occ_[pos.y][pos.x] = Occupancy::Free;
}

bool GridWorld::is_discovered(Pos pos) const {
    if (!in_bounds(pos)) return false;
    return discovered_[pos.y][pos.x] != 0;
}

void GridWorld::mark_discovered(Pos pos) {
    if (in_bounds(pos)) discovered_[pos.y][pos.x] = 1;
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
