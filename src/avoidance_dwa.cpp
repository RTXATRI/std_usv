#include "path_planning/avoidance_dwa.h"

#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace navigation {
namespace {
constexpr double kPi = 3.14159265358979323846;
double clamp(double value, double low, double high) { return std::max(low, std::min(value, high)); }
std::vector<double> samples(double low, double high, double step, double preferred) {
  std::vector<double> result;
  if (low > high) return result;
  result.push_back(low);
  for (double value = low + step; value < high - 1e-9; value += step) result.push_back(value);
  result.push_back(high);
  if (low <= 0 && high >= 0) result.push_back(0);
  if (preferred >= low && preferred <= high) result.push_back(preferred);
  std::sort(result.begin(), result.end());
  result.erase(std::unique(result.begin(), result.end(), [](double a, double b) {
    return std::abs(a - b) < 1e-8;
  }), result.end());
  return result;
}
Point bodyPoint(Point world, const State& state) {
  const double dx = world.x - state.x, dy = world.y - state.y;
  return {std::cos(state.yaw) * dx + std::sin(state.yaw) * dy,
          -std::sin(state.yaw) * dx + std::cos(state.yaw) * dy};
}
bool finiteState(const State& state) {
  return std::isfinite(state.x) && std::isfinite(state.y) && std::isfinite(state.yaw) &&
         std::isfinite(state.velocity) && std::isfinite(state.yawrate);
}
}  // namespace

void ObservationLayer::reset(std::size_t cells) {
  last_seen_.assign(cells, -std::numeric_limits<double>::infinity());
}
void ObservationLayer::observe(const AStar::Grid& known, const std::vector<Point>& points,
                               double stamp, double tolerance) {
  if (!known.valid() || !std::isfinite(stamp) || !std::isfinite(tolerance) || tolerance < 0)
    throw std::invalid_argument("Invalid observation layer geometry/time");
  if (last_seen_.size() != known.size()) reset(known.size());
  const int radius = int(std::ceil(tolerance / known.resolution()));
  for (const auto& point : points) {
    AStar::Cell cell;
    if (!known.worldToCell(point.x, point.y, cell)) continue;
    bool static_return = false;
    for (int dy = -radius; dy <= radius && !static_return; ++dy)
      for (int dx = -radius; dx <= radius; ++dx) {
        const AStar::Cell neighbour{cell.x + dx, cell.y + dy};
        if (std::hypot(dx, dy) * known.resolution() <= tolerance + 1e-9 && known.occupied(neighbour)) {
          static_return = true; break;
        }
      }
    if (!static_return)
      last_seen_[known.index(cell)] = std::max(last_seen_[known.index(cell)], stamp);
  }
}
AStar::Grid ObservationLayer::combined(const AStar::Grid& known, double now, double ttl) const {
  if (!known.valid() || last_seen_.size() != known.size() || !std::isfinite(now) ||
      !std::isfinite(ttl) || ttl <= 0) throw std::invalid_argument("Invalid observation layer lifetime");
  auto data = known.data();
  for (std::size_t i = 0; i < data.size(); ++i) {
    const double age = now - last_seen_[i];
    if (age >= -.05 && age <= ttl) data[i] = 1;
  }
  return AStar::Grid(known.width(), known.height(), known.resolution(), known.originX(), known.originY(), data);
}

bool Parameters::valid() const {
  const double values[] = {min_velocity, max_velocity, preferred_velocity, max_yawrate,
    max_acceleration, max_yaw_acceleration, velocity_resolution, yawrate_resolution,
    predict_time, simulation_dt, goal_gain, obstacle_gain, speed_gain, goal_bearing_gain,
    footprint_margin, max_obstacle_distance};
  for (double value : values) if (!std::isfinite(value)) return false;
  return min_velocity >= 0 && max_velocity > min_velocity && preferred_velocity >= min_velocity &&
      preferred_velocity <= max_velocity && max_yawrate > 0 && max_acceleration > 0 &&
      max_yaw_acceleration > 0 && velocity_resolution > 0 && yawrate_resolution > 0 &&
      predict_time > 0 && simulation_dt > 0 && simulation_dt <= .1 &&
      goal_gain > 0 && obstacle_gain >= 0 && speed_gain >= 0 && goal_bearing_gain >= 0 &&
      footprint_margin >= 0 && max_obstacle_distance > 0;
}
DwaCore::DwaCore(const Parameters& parameters, const std::vector<Point>& footprint)
    : params_(parameters), footprint_(footprint) {
  if (!params_.valid() || footprint_.size() < 3) throw std::invalid_argument("Invalid DWA parameters/footprint");
  min_x_ = max_x_ = footprint_[0].x; min_y_ = max_y_ = footprint_[0].y;
  double area = 0;
  for (std::size_t i = 0; i < footprint_.size(); ++i) {
    const auto& p = footprint_[i]; const auto& q = footprint_[(i + 1) % footprint_.size()];
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) throw std::invalid_argument("Non-finite footprint");
    radius_ = std::max(radius_, std::hypot(p.x, p.y));
    min_x_ = std::min(min_x_, p.x); max_x_ = std::max(max_x_, p.x);
    min_y_ = std::min(min_y_, p.y); max_y_ = std::max(max_y_, p.y);
    area += p.x * q.y - p.y * q.x;
  }
  if (std::abs(area) < 1e-6) throw std::invalid_argument("Degenerate footprint");
}
double DwaCore::normalizeAngle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

double DwaCore::polygonDistance(Point point, const std::vector<Point>& polygon) {
  if (polygon.size() < 3 || !std::isfinite(point.x) || !std::isfinite(point.y))
    return std::numeric_limits<double>::infinity();
  bool inside = false;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const Point a = polygon[j], b = polygon[i];
    const double dx = b.x - a.x, dy = b.y - a.y, length = dx * dx + dy * dy;
    const double t = length > 0 ? clamp(((point.x - a.x) * dx + (point.y - a.y) * dy) / length, 0, 1) : 0;
    best = std::min(best, std::hypot(point.x - a.x - t * dx, point.y - a.y - t * dy));
    if (((a.y > point.y) != (b.y > point.y)) &&
        point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
  }
  return inside ? 0 : best;
}
std::vector<Point> DwaCore::scanPoints(const std::vector<float>& ranges, double angle_min,
    double angle_increment, double range_min, double range_max,
    const std::vector<Point>& physical_footprint) {
  std::vector<Point> result;
  if (!std::isfinite(angle_min) || !std::isfinite(angle_increment) || angle_increment <= 0 ||
      !std::isfinite(range_min) || !std::isfinite(range_max) || range_min < 0 || range_max <= range_min)
    return result;
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double r = ranges[i];
    if (!std::isfinite(r) || r < range_min || r >= range_max) continue;
    const double angle = angle_min + i * angle_increment;
    const Point p{r * std::cos(angle), r * std::sin(angle)};
    if (polygonDistance(p, physical_footprint) <= 1e-6) continue;
    result.push_back(p);
  }
  return result;
}
std::vector<State> DwaCore::simulate(const State& start, double velocity, double yawrate, double horizon) const {
  std::vector<State> trajectory;
  if (!finiteState(start) || !std::isfinite(velocity) || !std::isfinite(yawrate)) return trajectory;
  if (horizon < 0) horizon = params_.predict_time;
  if (!std::isfinite(horizon) || horizon < 0) return trajectory;
  State state = start; state.velocity = velocity; state.yawrate = yawrate;
  trajectory.push_back(state);
  for (double time = 0; time < horizon - 1e-9;) {
    const double dt = std::min(params_.simulation_dt, horizon - time);
    // Midpoint integration avoids a yaw-dependent lateral discretization bias.
    state.x += velocity * std::cos(state.yaw + yawrate * dt * .5) * dt;
    state.y += velocity * std::sin(state.yaw + yawrate * dt * .5) * dt;
    state.yaw = normalizeAngle(state.yaw + yawrate * dt);
    trajectory.push_back(state); time += dt;
  }
  return trajectory;
}
bool DwaCore::collisionFree(const std::vector<State>& trajectory, const AStar::Grid& raw,
                           const std::vector<Point>& obstacles, double* clearance) const {
  if (!raw.valid() || trajectory.empty()) return false;
  double minimum = params_.max_obstacle_distance;
  const double padding = params_.footprint_margin;
  const double cell_radius = raw.resolution() * std::sqrt(2.0) * .5;
  const auto broadDistance = [this](Point p) {
    return std::hypot(std::max({min_x_ - p.x, 0.0, p.x - max_x_}),
                      std::max({min_y_ - p.y, 0.0, p.y - max_y_}));
  };
  for (const State& state : trajectory) {
    if (!finiteState(state)) return false;
    // Keep the entire footprint and its safety margin inside the surveyed grid.
    for (const Point& p : footprint_) {
      const Point v{state.x + std::cos(state.yaw) * p.x - std::sin(state.yaw) * p.y,
                    state.y + std::sin(state.yaw) * p.x + std::cos(state.yaw) * p.y};
      if (v.x - padding < raw.originX() || v.y - padding < raw.originY() ||
          v.x + padding >= raw.originX() + raw.width() * raw.resolution() ||
          v.y + padding >= raw.originY() + raw.height() * raw.resolution()) return false;
    }
    const int cx = int(std::floor((state.x - raw.originX()) / raw.resolution()));
    const int cy = int(std::floor((state.y - raw.originY()) / raw.resolution()));
    const int extent = int(std::ceil((radius_ + padding + cell_radius) / raw.resolution()));
    for (int y = cy - extent; y <= cy + extent; ++y) for (int x = cx - extent; x <= cx + extent; ++x) {
      const AStar::Cell cell{x, y};
      if (!raw.contains(cell) || !raw.occupied(cell)) continue;
      const Point body = bodyPoint({raw.centreX(cell), raw.centreY(cell)}, state);
      if (broadDistance(body) > padding + cell_radius) continue;
      if (polygonDistance(body, footprint_) <= padding + cell_radius)
        return false;
    }
    for (const Point& point : obstacles) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) continue;
      if (std::hypot(point.x - state.x, point.y - state.y) > radius_ + params_.max_obstacle_distance) continue;
      const Point body = bodyPoint(point, state);
      const double lower_bound = broadDistance(body);
      if (lower_bound > padding && lower_bound >= minimum) continue;
      const double distance = polygonDistance(body, footprint_);
      if (distance <= padding) return false;
      minimum = std::min(minimum, distance);
    }
  }
  if (clearance) *clearance = minimum;
  return true;
}
PlanResult DwaCore::plan(const State& start, const State& goal, const AStar::Grid& raw,
    const AStar::Grid& inflated, const std::vector<double>& field,
    const std::vector<Point>& obstacles, double control_dt) const {
  PlanResult best;
  if (!finiteState(start) || !finiteState(goal) || !std::isfinite(control_dt) || control_dt <= 0 ||
      !raw.valid() || !inflated.valid() || field.size() != inflated.size()) return best;
  // Wind, waves or feedback transients can put measured motion outside the
  // command limits. Project both window endpoints onto the legal interval so
  // that a nearest-boundary command can recover, rather than returning an
  // empty window and disabling the feedback controller with "blocked".
  // Keep the measured start state intact; only command samples are bounded.
  const double min_v = clamp(start.velocity - params_.max_acceleration * control_dt,
                             params_.min_velocity, params_.max_velocity);
  const double max_v = clamp(start.velocity + params_.max_acceleration * control_dt,
                             params_.min_velocity, params_.max_velocity);
  const double min_w = clamp(start.yawrate - params_.max_yaw_acceleration * control_dt,
                             -params_.max_yawrate, params_.max_yawrate);
  const double max_w = clamp(start.yawrate + params_.max_yaw_acceleration * control_dt,
                             -params_.max_yawrate, params_.max_yawrate);
  const auto velocities = samples(min_v, max_v, params_.velocity_resolution, params_.preferred_velocity);
  const auto yawrates = samples(min_w, max_w, params_.yawrate_resolution, 0);
  for (double v : velocities) for (double w : yawrates) {
    const auto trajectory = simulate(start, v, w);
    double clearance;
    if (!collisionFree(trajectory, raw, obstacles, &clearance)) continue;
    const State& end = trajectory.back();
    const double terminal = inflated.fieldValue(field, end.x, end.y);
    if (!std::isfinite(terminal)) continue;
    const double bearing_error = std::abs(normalizeAngle(std::atan2(goal.y - end.y, goal.x - end.x) - end.yaw));
    // A* in the original code used 10 units/metre. Keep that goal/obstacle scale
    // while storing the new reusable distance field in SI units.
    const double score = params_.goal_gain * 10.0 * terminal +
        params_.obstacle_gain / std::max(clearance, .01) +
        params_.speed_gain * std::abs(params_.preferred_velocity - v) + params_.goal_bearing_gain * bearing_error;
    best.candidates.push_back(trajectory);
    if (score < best.score - 1e-9 || (std::abs(score - best.score) <= 1e-9 && v > best.velocity)) {
      best.valid = true; best.score = score; best.velocity = v; best.yawrate = w; best.trajectory = trajectory;
    }
  }
  return best;
}

bool DwaCore::descentHeading(const State& start, const AStar::Grid& inflated,
    const std::vector<double>& field, double& heading) const {
  if (!finiteState(start) || !inflated.valid() || field.size() != inflated.size()) return false;
  const double here = inflated.fieldValue(field, start.x, start.y);
  if (!std::isfinite(here)) return false;
  // A short radial sample is a direction in the terminal-cost field, not a
  // waypoint on the published A* path. Check the entire ray so a lower-cost
  // cell across a wall cannot attract the recovery heading.
  const double radius = std::max(1.0, 2 * inflated.resolution());
  const double step = inflated.resolution() * .25;
  double best = here - .05;
  double turn = std::numeric_limits<double>::infinity();
  bool found = false;
  for (int sample = 0; sample < 64; ++sample) {
    const double angle = sample * 2 * kPi / 64;
    const double c = std::cos(angle), s = std::sin(angle);
    bool reachable = true;
    AStar::Cell previous;
    inflated.worldToCell(start.x, start.y, previous);
    for (double distance = step; distance <= radius + 1e-9; distance += step) {
      AStar::Cell cell;
      if (!inflated.worldToCell(start.x + c * distance, start.y + s * distance, cell) ||
          inflated.occupied(cell) ||
          (cell.x != previous.x && cell.y != previous.y &&
           (inflated.occupied({cell.x, previous.y}) || inflated.occupied({previous.x, cell.y})))) {
        reachable = false; break;
      }
      previous = cell;
    }
    if (!reachable) continue;
    const double value = inflated.fieldValue(field, start.x + c * radius, start.y + s * radius);
    const double rotation = std::abs(normalizeAngle(angle - start.yaw));
    if (std::isfinite(value) && (value < best - 1e-9 ||
        (found && std::abs(value - best) <= 1e-9 && rotation < turn))) {
      found = true; best = value; turn = rotation; heading = normalizeAngle(angle);
    }
  }
  return found;
}

PlanResult DwaCore::turnInPlace(const State& start, double heading, const AStar::Grid& raw,
    const std::vector<Point>& obstacles, double control_dt) const {
  PlanResult result;
  if (!finiteState(start) || !std::isfinite(heading) || !raw.valid() ||
      !std::isfinite(control_dt) || control_dt <= 0 || std::abs(start.velocity) > .05) return result;
  const double error = normalizeAngle(heading - start.yaw);
  const double desired = clamp(1.2 * error, -params_.max_yawrate, params_.max_yawrate);
  const double low = clamp(start.yawrate - params_.max_yaw_acceleration * control_dt,
                           -params_.max_yawrate, params_.max_yawrate);
  const double high = clamp(start.yawrate + params_.max_yaw_acceleration * control_dt,
                            -params_.max_yawrate, params_.max_yawrate);
  const double command = clamp(desired, low, high);
  // Reject a rotation whose full swept hull cannot fit, rather than discovering
  // a wall only after a tiny initial angular-velocity command has already moved.
  const int samples = std::max(1, int(std::ceil(std::abs(error) / .025)));
  std::vector<State> sweep;
  for (int i = 0; i <= samples; ++i) {
    State pose = start; pose.velocity = 0; pose.yawrate = command;
    pose.yaw = normalizeAngle(start.yaw + error * i / samples);
    sweep.push_back(pose);
  }
  if (!collisionFree(sweep, raw, obstacles)) return result;
  result.trajectory = simulate(start, 0, command, std::min(2.0, params_.predict_time));
  // Also test the immediate acceleration-limited command, including temporary
  // rotation in the old direction while braking an opposing measured yawrate.
  if (!collisionFree(result.trajectory, raw, obstacles)) {
    result.trajectory.clear(); return result;
  }
  result.valid = true; result.velocity = 0; result.yawrate = command;
  result.score = std::abs(error);
  return result;
}
}  // namespace navigation

namespace {
bool finitePose(const geometry_msgs::Pose& pose) {
  const auto& p = pose.position; const auto& q = pose.orientation;
  const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
      std::isfinite(norm) && norm > 1e-10;
}
bool fresh(const ros::Time& stamp, const ros::Time& now, double timeout) {
  const double age = (now - stamp).toSec();
  return !stamp.isZero() && age >= -.05 && age <= timeout;
}
double xmlNumber(const XmlRpc::XmlRpcValue& value) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) return int(value);
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) return double(value);
  throw std::invalid_argument("Footprint coordinates must be numeric");
}
}

NavigationCoordinator::NavigationCoordinator() : private_nh_("~") {
  private_nh_.param("HZ", hz_, 10.0);
  private_nh_.param("ROBOT_FRAME", robot_frame_, std::string("wamv/base_link"));
  private_nh_.param("PREFERRED_VELOCITY", params_.preferred_velocity, .8);
  private_nh_.param("MAX_VELOCITY", params_.max_velocity, 1.0);
  private_nh_.param("MIN_VELOCITY", params_.min_velocity, 0.0);
  private_nh_.param("MAX_YAWRATE", params_.max_yawrate, .5);
  private_nh_.param("MAX_ACCELERATION", params_.max_acceleration, .5);
  private_nh_.param("MAX_D_YAWRATE", params_.max_yaw_acceleration, .5);
  private_nh_.param("VELOCITY_RESOLUTION", params_.velocity_resolution, .1);
  private_nh_.param("YAWRATE_RESOLUTION", params_.yawrate_resolution, .05);
  private_nh_.param("PREDICT_TIME", params_.predict_time, 5.0);
  private_nh_.param("SIMULATION_DT", params_.simulation_dt, .1);
  private_nh_.param("TO_GOAL_COST_GAIN", params_.goal_gain, 1.0);
  private_nh_.param("OBSTACLE_COST_GAIN", params_.obstacle_gain, 25.0);
  private_nh_.param("SPEED_COST_GAIN", params_.speed_gain, 0.0);
  private_nh_.param("GOAL_BEARING_COST_GAIN", params_.goal_bearing_gain, 0.0);
  private_nh_.param("FOOTPRINT_MARGIN", params_.footprint_margin, .3);
  private_nh_.param("MAX_DIST", params_.max_obstacle_distance, 30.0);
  private_nh_.param("INFLATION_RADIUS", inflation_radius_, 2.0);
  private_nh_.param("INPUT_TIMEOUT", input_timeout_, .5);
  private_nh_.param("TRANSFORM_TIMEOUT", transform_timeout_, .1);
  private_nh_.param("GOAL_THRESHOLD", goal_threshold_, 1.0);
  private_nh_.param("GOAL_YAW_TOLERANCE", goal_yaw_tolerance_, .15);
  private_nh_.param("SETTLE_SECONDS", settle_seconds_, 2.0);
  private_nh_.param("SETTLE_VELOCITY_THRESHOLD", settle_velocity_threshold_, .05);
  private_nh_.param("SETTLE_YAWRATE_THRESHOLD", settle_yawrate_threshold_, .05);
  private_nh_.param("OBSERVATION_TTL", observation_ttl_, 5.0);
  private_nh_.param("OBSERVATION_REPLAN_INTERVAL", observation_replan_interval_, 1.0);
  private_nh_.param("STALL_TIMEOUT", stall_timeout_, 2.0);
  private_nh_.param("KNOWN_RETURN_TOLERANCE", known_return_tolerance_, .5);
  if (!params_.valid() || !std::isfinite(hz_) || hz_ <= 0 ||
      !std::isfinite(inflation_radius_) || inflation_radius_ < 0 ||
      !std::isfinite(input_timeout_) || input_timeout_ <= 0 ||
      !std::isfinite(transform_timeout_) || transform_timeout_ <= 0 || transform_timeout_ > input_timeout_ ||
      !std::isfinite(goal_threshold_) || goal_threshold_ <= 0 ||
      !std::isfinite(goal_yaw_tolerance_) || goal_yaw_tolerance_ <= 0 ||
      !std::isfinite(settle_seconds_) || settle_seconds_ < 0 ||
      !std::isfinite(settle_velocity_threshold_) || settle_velocity_threshold_ <= 0 ||
      !std::isfinite(settle_yawrate_threshold_) || settle_yawrate_threshold_ <= 0 ||
      !std::isfinite(observation_ttl_) || observation_ttl_ <= 0 ||
      !std::isfinite(observation_replan_interval_) || observation_replan_interval_ <= 0 ||
      !std::isfinite(stall_timeout_) || stall_timeout_ <= 0 ||
      !std::isfinite(known_return_tolerance_) || known_return_tolerance_ < 0)
    throw std::invalid_argument("Invalid planner limits, gains, timing or tolerances");
  footprint_valid_ = readFootprint();
  command_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  status_pub_ = nh_.advertise<std_msgs::String>("/wamv/navigation/status", 1, true);
  path_pub_ = nh_.advertise<nav_msgs::Path>("/wamv/navigation/global_path", 1, true);
  candidates_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
  selected_pub_ = private_nh_.advertise<visualization_msgs::Marker>("selected_trajectory", 1);
  map_sub_ = nh_.subscribe("/map", 1, &NavigationCoordinator::mapCallback, this);
  goal_sub_ = nh_.subscribe("/wamv/navigation/goal_request", 1, &NavigationCoordinator::goalCallback, this);
  scan_sub_ = nh_.subscribe("/scan", 1, &NavigationCoordinator::scanCallback, this);
  odom_sub_ = nh_.subscribe("/odom", 1, &NavigationCoordinator::odomCallback, this);
}
bool NavigationCoordinator::readFootprint() {
  XmlRpc::XmlRpcValue value;
  if (!nh_.getParam("/wamv/navigation/model/footprint", value)) {
    ROS_ERROR("Missing /wamv/navigation/model/footprint; navigation will remain stopped"); return false;
  }
  try {
    if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() < 3)
      throw std::invalid_argument("Footprint must have at least three vertices");
    for (int i = 0; i < value.size(); ++i) {
      if (value[i].getType() != XmlRpc::XmlRpcValue::TypeArray || value[i].size() != 2)
        throw std::invalid_argument("Footprint vertices must be [x, y]");
      footprint_.push_back({xmlNumber(value[i][0]), xmlNumber(value[i][1])});
    }
    navigation::DwaCore validation(params_, footprint_);
    return true;
  } catch (const std::exception& e) {
    ROS_ERROR("Invalid navigation footprint: %s", e.what()); footprint_.clear(); return false;
  }
}
void NavigationCoordinator::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
  try {
    const auto& info = msg->info;
    if (msg->header.frame_id.empty() || info.width > 10000 || info.height > 10000 ||
        std::size_t(info.width) * info.height > 10000000 ||
        !finitePose(info.origin) || std::abs(tf::getYaw(info.origin.orientation)) > 1e-6 ||
        std::abs(info.origin.orientation.x) > 1e-6 || std::abs(info.origin.orientation.y) > 1e-6)
      throw std::invalid_argument("Grid requires a finite, axis-aligned origin and frame");
    std::vector<std::uint8_t> occupied; occupied.reserve(msg->data.size());
    for (int value : msg->data) occupied.push_back(value < 0 || value >= 50);
    if (map_valid_ && raw_.width() == int(info.width) && raw_.height() == int(info.height) &&
        raw_.resolution() == info.resolution && raw_.originX() == info.origin.position.x &&
        raw_.originY() == info.origin.position.y && map_frame_ == msg->header.frame_id &&
        raw_.data() == occupied) return;
    raw_ = AStar::Grid(int(info.width), int(info.height), info.resolution,
                      info.origin.position.x, info.origin.position.y, occupied);
    known_inflated_ = raw_.inflated(inflation_radius_);
    routing_raw_ = raw_; inflated_ = known_inflated_;
    observations_.reset(raw_.size());
    last_observation_stamp_ = ros::Time(0); last_routing_update_ = ros::Time(0); stalled_since_ = ros::Time(0);
    map_frame_ = msg->header.frame_id;
    map_valid_ = true; dirty_ = true; field_.clear(); reached_ = false; settled_since_ = ros::Time(0);
    recovering_ = false;
    ROS_INFO("Navigation grid %dx%d at %.3f m/cell in %s", raw_.width(), raw_.height(), raw_.resolution(), map_frame_.c_str());
  } catch (const std::exception& e) {
    map_valid_ = false; dirty_ = true; field_.clear();
    recovering_ = false; stalled_since_ = ros::Time(0);
    ROS_ERROR("Rejected navigation grid: %s", e.what());
  }
}
void NavigationCoordinator::goalCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  // Every new message supersedes the old goal, including malformed messages.
  goal_valid_ = false; invalid_goal_ = true; dirty_ = true; reached_ = false;
  goal_pending_tf_ = false; pending_goal_ = geometry_msgs::PoseStamped{};
  recovering_ = false; stalled_since_ = ros::Time(0);
  settled_since_ = ros::Time(0); field_.clear(); publishPath(AStar::SearchResult{});
  if (msg->header.frame_id.empty() || !finitePose(msg->pose)) {
    ROS_WARN("Rejected non-finite goal, missing frame or zero quaternion");
    publishCommand(0, 0, "invalid_goal"); return;
  }
  pending_goal_ = *msg;
  const double norm = std::sqrt(pending_goal_.pose.orientation.x * pending_goal_.pose.orientation.x +
      pending_goal_.pose.orientation.y * pending_goal_.pose.orientation.y +
      pending_goal_.pose.orientation.z * pending_goal_.pose.orientation.z +
      pending_goal_.pose.orientation.w * pending_goal_.pose.orientation.w);
  pending_goal_.pose.orientation.x /= norm; pending_goal_.pose.orientation.y /= norm;
  pending_goal_.pose.orientation.z /= norm; pending_goal_.pose.orientation.w /= norm;
  goal_pending_tf_ = true; invalid_goal_ = false;
  if (!resolvePendingGoal()) publishCommand(0, 0, "waiting_for_inputs");
}
bool NavigationCoordinator::resolvePendingGoal() {
  if (!goal_pending_tf_) return goal_valid_;
  try {
    // Retain the normalized source pose until TF is available, then freeze the
    // resolved map pose once. Later motion of the source frame must not move
    // an already accepted goal. A new callback always replaces this pending
    // request, so invalid replacements cannot revive an older task.
    geometry_msgs::PoseStamped resolved = pending_goal_;
    if (resolved.header.frame_id != map_frame_)
      tf_.transformPose(map_frame_, pending_goal_, resolved);
    goal_ = resolved;
    goal_.header.frame_id = map_frame_;
    goal_valid_ = true; invalid_goal_ = false; goal_pending_tf_ = false;
    ROS_INFO("New goal: %.2f %.2f yaw %.2f in %s", goal_.pose.position.x,
             goal_.pose.position.y, tf::getYaw(goal_.pose.orientation), map_frame_.c_str());
    return true;
  } catch (const tf::TransformException& e) {
    ROS_WARN_THROTTLE(2, "Goal waits for TF: %s", e.what());
    return false;
  }
}
void NavigationCoordinator::scanCallback(const sensor_msgs::LaserScanConstPtr& msg) {
  scan_ = *msg;
  have_scan_ = msg->header.frame_id == robot_frame_ && !msg->ranges.empty() &&
      std::isfinite(msg->angle_min) && std::isfinite(msg->angle_increment) && msg->angle_increment > 0 &&
      std::isfinite(msg->range_min) && std::isfinite(msg->range_max) &&
      msg->range_min >= 0 && msg->range_max > msg->range_min;
  if (have_scan_) have_scan_ = std::any_of(msg->ranges.begin(), msg->ranges.end(), [&msg](float r) {
    return (std::isinf(r) && r > 0) || (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max);
  });
  if (!have_scan_) ROS_WARN_THROTTLE(2, "Invalid scan geometry/frame; expected %s", robot_frame_.c_str());
}
void NavigationCoordinator::odomCallback(const nav_msgs::OdometryConstPtr& msg) {
  odom_ = *msg;
  have_odom_ = finitePose(msg->pose.pose) && msg->child_frame_id == robot_frame_ &&
      std::isfinite(msg->twist.twist.linear.x) && std::isfinite(msg->twist.twist.angular.z);
}
void NavigationCoordinator::publishCommand(double velocity, double yawrate, const std::string& status) {
  geometry_msgs::Twist cmd; cmd.linear.x = velocity; cmd.angular.z = yawrate;
  command_pub_.publish(cmd);
  std_msgs::String state; state.data = status; status_pub_.publish(state);
}
void NavigationCoordinator::publishPath(const AStar::SearchResult& result) {
  nav_msgs::Path path; path.header.frame_id = map_frame_; path.header.stamp = ros::Time::now();
  for (const auto& cell : result.cells) {
    geometry_msgs::PoseStamped p; p.header = path.header;
    p.pose.position.x = inflated_.centreX(cell); p.pose.position.y = inflated_.centreY(cell);
    p.pose.orientation.w = 1; path.poses.push_back(p);
  }
  path_pub_.publish(path);
}
void NavigationCoordinator::publishTrajectories(const navigation::PlanResult& result) {
  visualization_msgs::MarkerArray candidates;
  visualization_msgs::Marker clear; clear.action = visualization_msgs::Marker::DELETEALL;
  candidates.markers.push_back(clear);
  auto marker = [this](const std::vector<navigation::State>& trajectory, int id, bool selected) {
    visualization_msgs::Marker line;
    line.header.frame_id = map_frame_; line.header.stamp = ros::Time::now();
    line.ns = selected ? "selected" : "candidates"; line.id = id;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1; line.scale.x = selected ? .09 : .025;
    line.color.a = selected ? 1 : .35; line.color.r = selected ? 1 : 0;
    line.color.g = selected ? .2 : 1; line.lifetime = ros::Duration(.5);
    for (const auto& state : trajectory) {
      geometry_msgs::Point p; p.x = state.x; p.y = state.y; p.z = .2; line.points.push_back(p);
    }
    return line;
  };
  int id = 0;
  for (const auto& trajectory : result.candidates) candidates.markers.push_back(marker(trajectory, id++, false));
  candidates_pub_.publish(candidates);
  selected_pub_.publish(marker(result.trajectory, 0, true));
}
void NavigationCoordinator::tick() {
  if (goal_pending_tf_ && !resolvePendingGoal()) {
    publishCommand(0, 0, "waiting_for_inputs"); return;
  }
  if (invalid_goal_) { publishCommand(0, 0, "invalid_goal"); return; }
  ros::Time now = ros::Time::now();
  if (!footprint_valid_ || !map_valid_ || !goal_valid_ || !have_scan_ || !have_odom_ ||
      !fresh(scan_.header.stamp, now, input_timeout_) || !fresh(odom_.header.stamp, now, input_timeout_)) {
    settled_since_ = ros::Time(0); stalled_since_ = ros::Time(0); recovering_ = false;
    publishCommand(0, 0, "waiting_for_inputs"); return;
  }
  tf::StampedTransform base, scan_transform;
  try {
    tf_.lookupTransform(map_frame_, robot_frame_, ros::Time(0), base);
    if (!fresh(base.stamp_, now, input_timeout_)) throw tf::TransformException("Stale robot TF");
    // The 10 Hz scan can precede the next 15 Hz localization transform. Allow
    // a full localization period for interpolation, within the freshness limit.
    if (!tf_.waitForTransform(map_frame_, robot_frame_, scan_.header.stamp, ros::Duration(transform_timeout_)))
      throw tf::TransformException("Scan-time robot transform unavailable");
    tf_.lookupTransform(map_frame_, robot_frame_, scan_.header.stamp, scan_transform);
    now = ros::Time::now();
    tf_.lookupTransform(map_frame_, robot_frame_, ros::Time(0), base);
    if (!fresh(base.stamp_, now, input_timeout_) ||
        !fresh(scan_.header.stamp, now, input_timeout_) ||
        !fresh(odom_.header.stamp, now, input_timeout_))
      throw tf::TransformException("Navigation data expired while waiting for scan-time TF");
  } catch (const tf::TransformException& e) {
    stalled_since_ = ros::Time(0); recovering_ = false;
    settled_since_ = ros::Time(0); ROS_WARN_THROTTLE(2, "Navigation waits for TF: %s", e.what());
    publishCommand(0, 0, "waiting_for_inputs"); return;
  }
  navigation::State current{base.getOrigin().x(), base.getOrigin().y(), tf::getYaw(base.getRotation()),
      odom_.twist.twist.linear.x, odom_.twist.twist.angular.z};
  navigation::State goal{goal_.pose.position.x, goal_.pose.position.y, tf::getYaw(goal_.pose.orientation), 0, 0};
  auto points = navigation::DwaCore::scanPoints(scan_.ranges, scan_.angle_min, scan_.angle_increment,
      scan_.range_min, std::min(double(scan_.range_max), params_.max_obstacle_distance), footprint_);
  for (auto& p : points) {
    const tf::Vector3 world = scan_transform * tf::Vector3(p.x, p.y, 0); p = {world.x(), world.y()};
  }
  if (scan_.header.stamp != last_observation_stamp_) {
    observations_.observe(raw_, points, scan_.header.stamp.toSec(), known_return_tolerance_);
    last_observation_stamp_ = scan_.header.stamp;
  }
  const double error = std::hypot(goal.x - current.x, goal.y - current.y);
  if (last_routing_update_.isZero() || (now - last_routing_update_).toSec() >= observation_replan_interval_) {
    const auto combined = observations_.combined(raw_, now.toSec(), observation_ttl_);
    if (combined.data() != routing_raw_.data()) {
      routing_raw_ = combined; inflated_ = routing_raw_.inflated(inflation_radius_); dirty_ = true;
    }
    last_routing_update_ = now;
  }
  AStar::Cell start_cell, goal_cell;
  if (!known_inflated_.worldToCell(goal.x, goal.y, goal_cell) || known_inflated_.occupied(goal_cell)) {
    goal_valid_ = false; invalid_goal_ = true; publishPath(AStar::SearchResult{});
    publishCommand(0, 0, "invalid_goal"); return;
  }
  // A moving obstacle may temporarily cover the goal. Keep the request so that
  // the route can recover when fresh observations/expiry clear the blockage.
  if (inflated_.occupied(goal_cell)) { publishCommand(0, 0, "blocked"); return; }
  if (!inflated_.worldToCell(current.x, current.y, start_cell) || inflated_.occupied(start_cell)) {
    publishCommand(0, 0, "blocked"); return;
  }
  if (dirty_) {
    // New requests/maps stop while establishing a route. Live sensor-layer
    // refreshes finish synchronously without pulsing the output gate every 1 s.
    if (field_.empty()) publishCommand(0, 0, "planning");
    AStar::Generator generator(inflated_);
    field_ = generator.distanceField(goal_cell);
    const auto path = generator.findPath(start_cell, goal_cell); publishPath(path);
    dirty_ = false;
    if (!path.found) { publishCommand(0, 0, "blocked"); return; }
  }
  if (field_.size() != inflated_.size() || !std::isfinite(field_[inflated_.index(start_cell)])) {
    publishCommand(0, 0, "blocked"); return;
  }
  if (reached_) { publishCommand(0, 0, "goal_reached"); return; }
  navigation::DwaCore core(params_, footprint_);
  const double yaw_error = navigation::DwaCore::normalizeAngle(goal.yaw - current.yaw);
  if (error <= goal_threshold_) {
    recovering_ = false; stalled_since_ = ros::Time(0);
    if (std::abs(yaw_error) <= goal_yaw_tolerance_) {
      // Include sway and leave a margin for drift after final thrust is zero.
      const auto& velocity = odom_.twist.twist.linear;
      if (std::hypot(velocity.x, velocity.y) <= settle_velocity_threshold_ &&
          std::abs(current.yawrate) <= settle_yawrate_threshold_) {
        if (settled_since_.isZero()) settled_since_ = now;
        reached_ = (now - settled_since_).toSec() >= settle_seconds_;
      } else settled_since_ = ros::Time(0);
      publishCommand(0, 0, reached_ ? "goal_reached" : "aligning"); return;
    }
    settled_since_ = ros::Time(0);
    double w = std::max(-params_.max_yawrate, std::min(params_.max_yawrate, 1.2 * yaw_error));
    w = std::max(current.yawrate - params_.max_yaw_acceleration / hz_,
                 std::min(current.yawrate + params_.max_yaw_acceleration / hz_, w));
    navigation::PlanResult result; result.trajectory = core.simulate(current, 0, w, std::min(2.0, params_.predict_time));
    if (!core.collisionFree(result.trajectory, raw_, points)) { publishCommand(0, 0, "blocked"); return; }
    publishTrajectories(result); publishCommand(0, w, "aligning"); return;
  }
  settled_since_ = ros::Time(0);
  if (recovering_) {
    const double turn_error = navigation::DwaCore::normalizeAngle(recovery_heading_ - current.yaw);
    if (std::abs(turn_error) < .2) {
      recovering_ = false; stalled_since_ = ros::Time(0);
    } else {
      const auto recovery = core.turnInPlace(current, recovery_heading_, raw_, points, 1.0 / hz_);
      publishTrajectories(recovery);
      if (!recovery.valid) {
        recovering_ = false; stalled_since_ = ros::Time(0);
        publishCommand(0, 0, "blocked"); return;
      }
      publishCommand(0, recovery.yawrate, "aligning"); return;
    }
  }
  const auto result = core.plan(current, goal, raw_, inflated_, field_, points, 1.0 / hz_);
  publishTrajectories(result);
  if (!result.valid) { stalled_since_ = ros::Time(0); publishCommand(0, 0, "blocked"); return; }
  const bool stopped = std::abs(current.velocity) < .05 && std::abs(current.yawrate) < .05 &&
      std::abs(result.velocity) < 1e-3 && std::abs(result.yawrate) < 1e-3;
  if (!stopped) stalled_since_ = ros::Time(0);
  else {
    if (stalled_since_.isZero()) stalled_since_ = now;
    if ((now - stalled_since_).toSec() >= stall_timeout_) {
      if (!core.descentHeading(current, inflated_, field_, recovery_heading_) ||
          std::abs(navigation::DwaCore::normalizeAngle(recovery_heading_ - current.yaw)) < .2) {
        publishCommand(0, 0, "blocked"); return;
      }
      const auto recovery = core.turnInPlace(current, recovery_heading_, raw_, points, 1.0 / hz_);
      publishTrajectories(recovery);
      if (!recovery.valid) { publishCommand(0, 0, "blocked"); return; }
      recovering_ = true; stalled_since_ = ros::Time(0);
      ROS_INFO("Recovering a stationary DWA heading using descending field direction %.2f rad",
               recovery_heading_);
      publishCommand(0, recovery.yawrate, "aligning"); return;
    }
  }
  publishCommand(result.velocity, result.yawrate, "navigating");
}
void NavigationCoordinator::process() {
  ros::Rate rate(hz_);
  ros::Time previous = ros::Time::now();
  while (ros::ok()) {
    ros::spinOnce();
    const ros::Time now = ros::Time::now();
    if (now < previous) {
      // Keep the world-fixed goal/map, but require sensor/TF samples from the new
      // clock epoch and re-evaluate reachability from the reset boat position.
      have_scan_ = false; have_odom_ = false; dirty_ = true; reached_ = false;
      settled_since_ = ros::Time(0); stalled_since_ = ros::Time(0);
      recovering_ = false;
      last_observation_stamp_ = ros::Time(0); last_routing_update_ = ros::Time(0);
      observations_.reset(raw_.size()); routing_raw_ = raw_; inflated_ = known_inflated_;
      field_.clear(); tf_.clear();
      ROS_WARN("Simulation clock moved backwards; navigation waits for fresh inputs");
    }
    previous = now;
    tick(); rate.sleep();
  }
  publishCommand(0, 0, "waiting_for_inputs");
}
