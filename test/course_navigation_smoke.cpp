// Deterministic algorithm regression, not a Gazebo/WAM-V dynamics test.
// The planner sees only the surveyed grid and simulated 180-beam lidar returns.
// Full unknown-obstacle geometry is used separately as a collision oracle.
#include "path_planning/avoidance_dwa.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
constexpr double kDt = .1;
constexpr double kLag = .6;
constexpr double kPi = 3.14159265358979323846;

struct Box {
  double min_x = 0, min_y = 0, max_x = 0, max_y = 0;
  bool contains(navigation::Point point) const {
    return point.x >= min_x && point.x <= max_x &&
           point.y >= min_y && point.y <= max_y;
  }
};

bool separated(const std::vector<navigation::Point>& hull, const Box& box,
               navigation::Point axis) {
  double lo = 1e100, hi = -1e100;
  for (const auto& point : hull) {
    const double projection = point.x * axis.x + point.y * axis.y;
    lo = std::min(lo, projection);
    hi = std::max(hi, projection);
  }
  const double centre = .5 * (box.min_x + box.max_x) * axis.x +
                        .5 * (box.min_y + box.max_y) * axis.y;
  const double radius = .5 * (box.max_x - box.min_x) * std::abs(axis.x) +
                        .5 * (box.max_y - box.min_y) * std::abs(axis.y);
  return hi < centre - radius || lo > centre + radius;
}

bool intersects(const navigation::State& state,
                const std::vector<navigation::Point>& footprint, const Box& box) {
  std::vector<navigation::Point> hull;
  const double c = std::cos(state.yaw), s = std::sin(state.yaw);
  for (const auto& point : footprint)
    hull.push_back({state.x + c * point.x - s * point.y,
                    state.y + s * point.x + c * point.y});
  if (separated(hull, box, {1, 0}) || separated(hull, box, {0, 1})) return false;
  for (std::size_t i = 0; i < hull.size(); ++i) {
    const auto& a = hull[i];
    const auto& b = hull[(i + 1) % hull.size()];
    if (separated(hull, box, {a.y - b.y, b.x - a.x})) return false;
  }
  return true;
}

std::vector<navigation::Point> scan(const navigation::State& state,
    const AStar::Grid& known, const std::vector<Box>& unknown) {
  std::vector<navigation::Point> points;
  for (int beam = 0; beam < 180; ++beam) {
    const double angle = beam * 2 * kPi / 180;
    const double c = std::cos(angle), s = std::sin(angle);
    for (double range = .25; range <= 30; range += .25) {
      const navigation::Point point{state.x + c * range, state.y + s * range};
      AStar::Cell cell;
      bool hit = !known.worldToCell(point.x, point.y, cell) || known.occupied(cell);
      if (!hit)
        hit = std::any_of(unknown.begin(), unknown.end(),
                         [&](const Box& box) { return box.contains(point); });
      if (hit) {
        points.push_back(point);
        break;
      }
    }
  }
  return points;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: course_navigation_smoke MAP_AND_FOOTPRINT UNKNOWN_BOXES\n";
    return 1;
  }
  try {
    std::ifstream map_file(argv[1]);
    int width = 0, height = 0;
    double resolution = 0, origin_x = 0, origin_y = 0;
    map_file >> width >> height >> resolution >> origin_x >> origin_y;
    if (!map_file || width <= 0 || height <= 0 || width > 10000 || height > 10000)
      throw std::runtime_error("Invalid map header");
    std::vector<std::uint8_t> occupied(std::size_t(width) * height);
    for (auto& value : occupied) {
      int parsed = 0;
      map_file >> parsed;
      value = parsed != 0;
    }
    int count = 0;
    map_file >> count;
    if (!map_file || count < 3 || count > 10000)
      throw std::runtime_error("Invalid footprint header");
    std::vector<navigation::Point> footprint(count);
    for (auto& point : footprint) map_file >> point.x >> point.y;
    if (!map_file) throw std::runtime_error("Incomplete map/footprint input");
    std::ifstream unknown_file(argv[2]);
    unknown_file >> count;
    if (!unknown_file || count < 0 || count > 10000)
      throw std::runtime_error("Invalid unknown obstacle header");
    std::vector<Box> unknown(count);
    for (auto& box : unknown) {
      unknown_file >> box.min_x >> box.min_y >> box.max_x >> box.max_y;
      if (box.min_x >= box.max_x || box.min_y >= box.max_y)
        throw std::runtime_error("Invalid obstacle bounds");
    }
    if (!unknown_file) throw std::runtime_error("Incomplete obstacle input");

    const AStar::Grid known(width, height, resolution, origin_x, origin_y, occupied);
    auto routing = known;
    auto inflated = routing.inflated(2);
    const navigation::State goal{-500, 232, 0, 0, 0};
    navigation::State state{-570, 167, 1, 0, 0};
    AStar::Cell goal_cell;
    if (!inflated.worldToCell(goal.x, goal.y, goal_cell))
      throw std::runtime_error("Goal is outside map");
    auto field = AStar::Generator(inflated).distanceField(goal_cell);
    navigation::ObservationLayer observations;
    observations.reset(known.size());
    const navigation::Parameters parameters;
    const navigation::DwaCore planner(parameters, footprint);
    const auto start = std::chrono::steady_clock::now();
    int replans = 0;
    double planning_ms = 0;
    for (int step = 0; step < 6000; ++step) {
      const double now = step * kDt;
      const auto returns = scan(state, known, unknown);
      observations.observe(known, returns, now, .5);
      if (step % 10 == 0) {
        const auto combined = observations.combined(known, now, 5);
        if (combined.data() != routing.data()) {
          routing = combined;
          inflated = routing.inflated(2);
          field = AStar::Generator(inflated).distanceField(goal_cell);
          ++replans;
        }
      }
      const auto before = std::chrono::steady_clock::now();
      const auto result = planner.plan(state, goal, known, inflated, field, returns, kDt);
      planning_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - before).count();
      const double command_v = result.valid ? result.velocity : 0;
      const double command_w = result.valid ? result.yawrate : 0;
      // Simple actuator lag; this is not an identification of WAM-V dynamics.
      const double actual_v = state.velocity + (command_v - state.velocity) * kDt / kLag;
      const double actual_w = state.yawrate + (command_w - state.yawrate) * kDt / kLag;
      state = planner.simulate(state, actual_v, actual_w, kDt).back();
      const bool unknown_collision = std::any_of(unknown.begin(), unknown.end(),
          [&](const Box& box) { return intersects(state, footprint, box); });
      if (unknown_collision || !planner.collisionFree({state}, known, returns)) {
        std::cerr << "Footprint collision or safety-margin violation at " << now + kDt
                  << "s: " << state.x << ", " << state.y << '\n';
        return 4;
      }
      const double error = std::hypot(state.x - goal.x, state.y - goal.y);
      if (error <= 1) {
        std::cout << "{\"position_error\":" << error
                  << ",\"sim_seconds\":" << now + kDt
                  << ",\"replans\":" << replans
                  << ",\"mean_plan_ms\":" << planning_ms / (step + 1)
                  << ",\"wall_seconds\":" << std::chrono::duration<double>(
                      std::chrono::steady_clock::now() - start).count()
                  << ",\"collision_free\":true}" << std::endl;
        return 0;
      }
    }
    std::cerr << "Goal was not reached within 600 simulated seconds: "
              << state.x << ", " << state.y << '\n';
    return 3;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
