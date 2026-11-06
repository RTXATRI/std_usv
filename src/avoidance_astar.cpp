#include "path_planning/avoidance_astar.h"
#include <algorithm>
#include <cmath>
#include <queue>
#include <stdexcept>
#include <utility>

namespace AStar {
namespace {
const Cell kDirections[] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
double distance(Cell a, Cell b) { return std::hypot(double(a.x - b.x), double(a.y - b.y)); }
using QueueItem = std::pair<double, std::size_t>;
using Queue = std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>>;
}
Grid::Grid(int width, int height, double resolution, double origin_x, double origin_y,
           const std::vector<std::uint8_t>& occupied)
    : width_(width), height_(height), resolution_(resolution), origin_x_(origin_x),
      origin_y_(origin_y), occupied_(occupied) {
  if (!valid()) throw std::invalid_argument("Invalid navigation grid geometry/data");
}
bool Grid::valid() const {
  return width_ > 0 && height_ > 0 && std::isfinite(resolution_) && resolution_ > 0 &&
      std::isfinite(origin_x_) && std::isfinite(origin_y_) &&
      occupied_.size() == static_cast<std::size_t>(width_) * height_;
}
bool Grid::contains(Cell c) const { return c.x >= 0 && c.y >= 0 && c.x < width_ && c.y < height_; }
bool Grid::occupied(Cell c) const { return !contains(c) || occupied_[index(c)] != 0; }
std::size_t Grid::index(Cell c) const { return static_cast<std::size_t>(c.y) * width_ + c.x; }
Cell Grid::cell(std::size_t i) const { return {int(i % width_), int(i / width_)}; }
bool Grid::worldToCell(double x, double y, Cell& c) const {
  if (!valid() || !std::isfinite(x) || !std::isfinite(y)) return false;
  const double cx = std::floor((x - origin_x_) / resolution_), cy = std::floor((y - origin_y_) / resolution_);
  if (cx < 0 || cy < 0 || cx >= width_ || cy >= height_) return false;
  c = {int(cx), int(cy)}; return true;
}
double Grid::centreX(Cell c) const { return origin_x_ + (c.x + 0.5) * resolution_; }
double Grid::centreY(Cell c) const { return origin_y_ + (c.y + 0.5) * resolution_; }
Grid Grid::inflated(double radius) const {
  if (!valid() || !std::isfinite(radius) || radius < 0) throw std::invalid_argument("Invalid grid/inflation radius");
  Grid result(*this);
  const int n = int(std::ceil(radius / resolution_));
  for (int y = 0; y < height_; ++y) for (int x = 0; x < width_; ++x) {
    if (!occupied({x, y})) continue;
    for (int dy = -n; dy <= n; ++dy) for (int dx = -n; dx <= n; ++dx) {
      const Cell c{x + dx, y + dy};
      if (contains(c) && std::hypot(dx, dy) * resolution_ <= radius + 1e-9)
        result.occupied_[index(c)] = 1;
    }
  }
  for (int y = 0; y < height_; ++y) for (int x = 0; x < width_; ++x)
    if (std::min({x + .5, y + .5, width_ - x - .5, height_ - y - .5}) * resolution_ < radius)
      result.occupied_[index({x, y})] = 1;
  return result;
}
double Grid::fieldValue(const std::vector<double>& field, double x, double y) const {
  Cell c;
  if (field.size() != size() || !worldToCell(x, y, c) || occupied(c))
    return std::numeric_limits<double>::infinity();
  const double fx = (x - origin_x_) / resolution_ - .5, fy = (y - origin_y_) / resolution_ - .5;
  const int x0 = int(std::floor(fx)), y0 = int(std::floor(fy));
  const double wx = fx - x0, wy = fy - y0;
  double sum = 0, weights = 0;
  for (int dy = 0; dy <= 1; ++dy) for (int dx = 0; dx <= 1; ++dx) {
    const Cell n{x0 + dx, y0 + dy};
    const double w = (dx ? wx : 1 - wx) * (dy ? wy : 1 - wy);
    if (w <= 1e-12) continue;
    if (!contains(n) || occupied(n) || !std::isfinite(field[index(n)])) return field[index(c)];
    sum += w * field[index(n)]; weights += w;
  }
  return weights > 0 ? sum / weights : field[index(c)];
}
Generator::Generator(const Grid& grid) : grid_(grid) {
  if (!grid_.valid()) throw std::invalid_argument("A* requires a valid grid");
}
bool Generator::traversable(Cell from, Cell to) const {
  if (grid_.occupied(to)) return false;
  if (from.x != to.x && from.y != to.y)
    return !grid_.occupied({from.x, to.y}) && !grid_.occupied({to.x, from.y});
  return true;
}
SearchResult Generator::findPath(Cell start, Cell goal) const {
  SearchResult result;
  if (grid_.occupied(start) || grid_.occupied(goal)) return result;
  std::vector<double> g(grid_.size(), std::numeric_limits<double>::infinity());
  std::vector<std::size_t> parent(grid_.size(), grid_.size());
  Queue queue;
  const auto s = grid_.index(start), t = grid_.index(goal);
  g[s] = 0; queue.push({distance(start, goal) * grid_.resolution(), s});
  while (!queue.empty()) {
    const auto item = queue.top(); queue.pop();
    const Cell here = grid_.cell(item.second);
    if (item.first > g[item.second] + distance(here, goal) * grid_.resolution() + 1e-9) continue;
    if (item.second == t) {
      result.found = true; result.cost = g[t];
      for (auto i = t; i != grid_.size(); i = parent[i]) {
        result.cells.push_back(grid_.cell(i)); if (i == s) break;
      }
      std::reverse(result.cells.begin(), result.cells.end()); return result;
    }
    for (const auto& d : kDirections) {
      const Cell next{here.x + d.x, here.y + d.y};
      if (!traversable(here, next)) continue;
      const auto ni = grid_.index(next);
      const double candidate = g[item.second] + distance(here, next) * grid_.resolution();
      if (candidate + 1e-9 < g[ni]) {
        g[ni] = candidate; parent[ni] = item.second;
        queue.push({candidate + distance(next, goal) * grid_.resolution(), ni});
      }
    }
  }
  return result;
}
std::vector<double> Generator::distanceField(Cell goal) const {
  std::vector<double> costs(grid_.size(), std::numeric_limits<double>::infinity());
  if (grid_.occupied(goal)) return costs;
  Queue queue; costs[grid_.index(goal)] = 0; queue.push({0, grid_.index(goal)});
  while (!queue.empty()) {
    const auto item = queue.top(); queue.pop();
    if (item.first > costs[item.second] + 1e-9) continue;
    const Cell here = grid_.cell(item.second);
    for (const auto& d : kDirections) {
      const Cell next{here.x + d.x, here.y + d.y};
      if (!traversable(here, next)) continue;
      const auto ni = grid_.index(next);
      const double candidate = item.first + distance(here, next) * grid_.resolution();
      if (candidate + 1e-9 < costs[ni]) { costs[ni] = candidate; queue.push({candidate, ni}); }
    }
  }
  return costs;
}
}  // namespace AStar
