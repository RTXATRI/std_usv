#ifndef STD_USV_AVOIDANCE_DWA_H
#define STD_USV_AVOIDANCE_DWA_H

#include "path_planning/avoidance_astar.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <string>
#include <vector>

namespace navigation {
struct Point { double x = 0, y = 0; };
struct State { double x = 0, y = 0, yaw = 0, velocity = 0, yawrate = 0; };
// A short-lived routing layer built exclusively from sensor observations.
// The surveyed/static grid is never modified by this layer.
class ObservationLayer {
 public:
  void reset(std::size_t cells);
  void observe(const AStar::Grid& known, const std::vector<Point>& points,
               double stamp, double known_return_tolerance = .5);
  AStar::Grid combined(const AStar::Grid& known, double now, double ttl) const;
 private:
  std::vector<double> last_seen_;
};
struct Parameters {
  // Preferred forward velocity is sampled and optionally scored, not a limit.
  double min_velocity = 0, max_velocity = 1, preferred_velocity = .8;
  double max_yawrate = .5, max_acceleration = .5, max_yaw_acceleration = .5;
  double velocity_resolution = .1, yawrate_resolution = .05;
  double predict_time = 5, simulation_dt = .1;
  // Bearing points toward the goal position; final goal yaw is checked separately.
  double goal_gain = 1, obstacle_gain = 25, speed_gain = 0, goal_bearing_gain = 0;
  double footprint_margin = .3, max_obstacle_distance = 30;
  bool valid() const;
};
struct PlanResult {
  bool valid = false;
  double velocity = 0, yawrate = 0;
  double score = std::numeric_limits<double>::infinity();
  std::vector<State> trajectory;
  std::vector<std::vector<State>> candidates;
};
class DwaCore {
 public:
  DwaCore(const Parameters& parameters, const std::vector<Point>& footprint);
  PlanResult plan(const State& start, const State& goal, const AStar::Grid& raw,
                  const AStar::Grid& inflated, const std::vector<double>& field,
                  const std::vector<Point>& obstacles, double control_dt) const;
  // Stagnation recovery uses the same terminal field, without path tracking or
  // changing the ordinary DWA objective. Both helpers are ROS-independent.
  bool descentHeading(const State& start, const AStar::Grid& inflated,
                      const std::vector<double>& field, double& heading) const;
  PlanResult turnInPlace(const State& start, double heading, const AStar::Grid& raw,
                        const std::vector<Point>& obstacles, double control_dt) const;
  std::vector<State> simulate(const State& start, double velocity, double yawrate,
                              double horizon = -1) const;
  bool collisionFree(const std::vector<State>& trajectory, const AStar::Grid& raw,
                     const std::vector<Point>& obstacles, double* clearance = nullptr) const;
  static double polygonDistance(Point point, const std::vector<Point>& polygon);
  static std::vector<Point> scanPoints(const std::vector<float>& ranges, double angle_min,
      double angle_increment, double range_min, double range_max,
      const std::vector<Point>& physical_footprint);
  static double normalizeAngle(double angle);
 private:
  Parameters params_;
  std::vector<Point> footprint_;
  double radius_ = 0;
  double min_x_ = 0, max_x_ = 0, min_y_ = 0, max_y_ = 0;
};
}  // namespace navigation

// Coordinates ROS inputs, global routing, DWA, recovery and goal completion.
// Fixed-frame goals/map and body-frame twist/scan, with no ground-truth input.
class NavigationCoordinator {
 public:
  NavigationCoordinator();
  void process();
 private:
  void mapCallback(const nav_msgs::OccupancyGridConstPtr&);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr&);
  bool resolvePendingGoal();
  void scanCallback(const sensor_msgs::LaserScanConstPtr&);
  void odomCallback(const nav_msgs::OdometryConstPtr&);
  void tick();
  void publishCommand(double velocity, double yawrate, const std::string& status);
  void publishPath(const AStar::SearchResult& result);
  void publishTrajectories(const navigation::PlanResult& result);
  bool readFootprint();
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher command_pub_, status_pub_, path_pub_, candidates_pub_, selected_pub_;
  ros::Subscriber map_sub_, goal_sub_, scan_sub_, odom_sub_;
  tf::TransformListener tf_;
  navigation::Parameters params_;
  std::vector<navigation::Point> footprint_;
  AStar::Grid raw_, known_inflated_, routing_raw_, inflated_;
  navigation::ObservationLayer observations_;
  std::vector<double> field_;
  sensor_msgs::LaserScan scan_;
  nav_msgs::Odometry odom_;
  geometry_msgs::PoseStamped goal_, pending_goal_;
  std::string map_frame_ = "map", robot_frame_ = "wamv/base_link";
  bool footprint_valid_ = false, map_valid_ = false, goal_valid_ = false;
  bool have_scan_ = false, have_odom_ = false, dirty_ = true;
  bool invalid_goal_ = false, reached_ = false;
  bool goal_pending_tf_ = false;
  bool recovering_ = false;
  double recovery_heading_ = 0;
  double hz_ = 10, inflation_radius_ = 2, input_timeout_ = .5;
  double transform_timeout_ = .1;
  double goal_threshold_ = 1, goal_yaw_tolerance_ = .15, settle_seconds_ = 2;
  double settle_velocity_threshold_ = .05, settle_yawrate_threshold_ = .05;
  double observation_ttl_ = 5, observation_replan_interval_ = 1, stall_timeout_ = 2;
  double known_return_tolerance_ = .5;
  ros::Time settled_since_, last_observation_stamp_, last_routing_update_, stalled_since_;
};
#endif
