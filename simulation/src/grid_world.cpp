#include "grid_world.hpp"


GridWorld::GridWorld(int w, int h)
  : w_(w), h_(h), blocked_(h, std::vector<unsigned char>(w, 0)) {}


int GridWorld::width() const { return w_; }
int GridWorld::height() const { return h_; }

bool GridWorld::in_bounds(int x, int y) const {
  return 0 <= x && x < w_ && 0 <= y && y < h_;
}

// --- occupancy -----
void GridWorld::set_blocked(int x, int y, bool blocked) {
  if (!in_bounds(x,y)) return;
  blocked_[y][x] = blocked ? 1 : 0;
}

bool GridWorld::is_blocked(int x, int y) const {
  if (!in_bounds(x,y)) return true;
  return blocked_[y][x] != 0;
}

std::vector<GridWorld::Pos>
GridWorld::neighbors4(int x, int y) const {
  std::vector<Pos> nb;
  if (is_blocked(x,y)) return nb;

  const int dx[4] = { 1, -1,  0,  0 };
  const int dy[4] = { 0,  0,  1, -1 };

  for (int k = 0; k < 4; ++k) {
    int nx = x + dx[k], ny = y + dy[k];
    if (in_bounds(nx, ny) && !is_blocked(nx, ny))
      nb.emplace_back(nx, ny);
  }
  return nb;
}
