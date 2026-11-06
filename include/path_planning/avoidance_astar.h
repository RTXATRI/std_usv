#ifndef STD_USV_AVOIDANCE_ASTAR_H
#define STD_USV_AVOIDANCE_ASTAR_H
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

// Row-major cells and cell-centred samples; distances and world poses use metres.
namespace AStar {
struct Cell {
  int x = 0;
  int y = 0;
  bool operator==(const Cell& other) const { return x == other.x && y == other.y; }
};
class Grid {
 public:
  Grid() = default;
  Grid(int width, int height, double resolution, double origin_x, double origin_y,
       const std::vector<std::uint8_t>& occupied);
  bool valid() const;
  bool contains(Cell cell) const;
  bool occupied(Cell cell) const;
  bool worldToCell(double x, double y, Cell& cell) const;
  std::size_t index(Cell cell) const;
  Cell cell(std::size_t index) const;
  double centreX(Cell cell) const;
  double centreY(Cell cell) const;
  Grid inflated(double radius) const;
  double fieldValue(const std::vector<double>& field, double x, double y) const;
  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }
  double originX() const { return origin_x_; }
  double originY() const { return origin_y_; }
  std::size_t size() const { return occupied_.size(); }
  const std::vector<std::uint8_t>& data() const { return occupied_; }
 private:
  int width_ = 0, height_ = 0;
  double resolution_ = 0.0, origin_x_ = 0.0, origin_y_ = 0.0;
  std::vector<std::uint8_t> occupied_;
};
struct SearchResult {
  bool found = false;
  double cost = std::numeric_limits<double>::infinity();
  std::vector<Cell> cells;
};
class Generator {
 public:
  explicit Generator(const Grid& grid);
  SearchResult findPath(Cell start, Cell goal) const;
  // Complete goal-rooted shortest distance; unreachable cells are infinity.
  std::vector<double> distanceField(Cell goal) const;
 private:
  bool traversable(Cell from, Cell to) const;
  Grid grid_;
};
}  // namespace AStar
#endif
