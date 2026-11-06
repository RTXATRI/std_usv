#include "path_planning/avoidance_astar.h"
#include "path_planning/avoidance_dwa.h"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

namespace {
AStar::Grid emptyGrid(int width = 40, int height = 40, double resolution = .5,
                     double x = -10, double y = -10) {
  return AStar::Grid(width, height, resolution, x, y,
                     std::vector<std::uint8_t>(width * height, 0));
}
std::vector<navigation::Point> footprint() { return {{-.7, -.4}, {.7, -.4}, {.7, .4}, {-.7, .4}}; }
}
TEST(Grid, NegativeWorldCoordinatesAndUpperEdges) {
  const auto grid = emptyGrid(); AStar::Cell c;
  ASSERT_TRUE(grid.worldToCell(-9.75, -9.75, c)); EXPECT_EQ(c.x, 0); EXPECT_EQ(c.y, 0);
  EXPECT_FALSE(grid.worldToCell(-10.01, 0, c)); EXPECT_FALSE(grid.worldToCell(10, 0, c));
  ASSERT_TRUE(grid.worldToCell(9.999, 9.999, c)); EXPECT_EQ(c.x, 39); EXPECT_EQ(c.y, 39);
  EXPECT_TRUE(grid.occupied({-1, 3})); EXPECT_TRUE(grid.occupied({40, 3}));
  EXPECT_FALSE(grid.worldToCell(std::numeric_limits<double>::quiet_NaN(), 0, c));
}
TEST(Grid, RejectsInconsistentGeometry) {
  EXPECT_THROW(AStar::Grid(5, 5, .5, 0, 0, std::vector<std::uint8_t>(4)), std::invalid_argument);
  EXPECT_THROW(AStar::Grid(2, 2, 0, 0, 0, std::vector<std::uint8_t>(4)), std::invalid_argument);
}
TEST(Search, CannotCutBlockedDiagonalOrReturnPartialPath) {
  auto data = std::vector<std::uint8_t>(9, 0); data[1] = 1; data[3] = 1;
  const AStar::Grid grid(3, 3, 1, 0, 0, data);
  const AStar::Generator generator(grid);
  auto result = generator.findPath({0, 0}, {2, 2});
  EXPECT_FALSE(result.found); EXPECT_TRUE(result.cells.empty()); EXPECT_FALSE(std::isfinite(result.cost));
  EXPECT_FALSE(std::isfinite(generator.distanceField({2, 2})[grid.index({0, 0})]));
}
TEST(Search, GoalFieldCompleteAndReplacedForNewGoal) {
  const auto grid = emptyGrid(7, 7, 1, -3, -3); AStar::Generator generator(grid);
  auto a = generator.distanceField({0, 0}); auto b = generator.distanceField({6, 6});
  EXPECT_DOUBLE_EQ(a[grid.index({0, 0})], 0); EXPECT_DOUBLE_EQ(b[grid.index({6, 6})], 0);
  EXPECT_NEAR(a[grid.index({6, 6})], 6 * std::sqrt(2.0), 1e-8);
  for (std::size_t i = 0; i < grid.size(); ++i) EXPECT_TRUE(std::isfinite(a[i]));
  const auto path = generator.findPath({0, 0}, {6, 6});
  ASSERT_TRUE(path.found); ASSERT_FALSE(path.cells.empty());
  EXPECT_EQ(path.cells.front().x, 0); EXPECT_EQ(path.cells.back().x, 6);
  EXPECT_NEAR(path.cost, a[grid.index({6, 6})], 1e-8);
}
TEST(Search, InflationKeepsObstaclesAndBoundaryOutOfSearch) {
  std::vector<std::uint8_t> data(100, 0); data[55] = 1;
  const auto inflated = AStar::Grid(10, 10, 1, 0, 0, data).inflated(2);
  EXPECT_TRUE(inflated.occupied({5, 5})); EXPECT_TRUE(inflated.occupied({7, 5}));
  EXPECT_TRUE(inflated.occupied({0, 4})); EXPECT_FALSE(inflated.occupied({2, 2}));
}
TEST(Search, InterpolationHandlesMapBorderAndUnreachableValues) {
  const auto grid = emptyGrid(5, 5, .5, -1, -1);
  auto field = AStar::Generator(grid).distanceField({4, 4});
  EXPECT_TRUE(std::isfinite(grid.fieldValue(field, -.99, -.99)));
  EXPECT_FALSE(std::isfinite(grid.fieldValue(field, 1.5, 0)));
  EXPECT_DOUBLE_EQ(grid.fieldValue(field, grid.centreX({4, 4}), grid.centreY({4, 4})), 0);
  EXPECT_FALSE(std::isfinite(grid.fieldValue({}, 0, 0)));
}
TEST(Scan, FiltersSelfReturnsNaNInfinityAndOutOfRange) {
  const float nan = std::numeric_limits<float>::quiet_NaN(), inf = std::numeric_limits<float>::infinity();
  const auto points = navigation::DwaCore::scanPoints({nan, inf, -.1f, .2f, 3.f, 31.f}, 0, .1, .1, 30, footprint());
  ASSERT_EQ(points.size(), 1u); EXPECT_NEAR(std::hypot(points[0].x, points[0].y), 3, 1e-6);
  EXPECT_TRUE(navigation::DwaCore::scanPoints({inf, inf}, 0, .1, .1, 30, footprint()).empty());
}
TEST(Dwa, RejectsFootprintCollisionIncludingRotationAndKnownCells) {
  navigation::Parameters p; navigation::DwaCore core(p, footprint());
  auto grid = emptyGrid();
  EXPECT_FALSE(core.collisionFree({{0, 0, 0, 0, 0}}, grid, {{.9, 0}}));
  EXPECT_TRUE(core.collisionFree({{0, 0, 0, 0, 0}}, grid, {{2, 0}}));
  EXPECT_FALSE(core.collisionFree({{0, 0, 1.57079632679, 0, 0}}, grid, {{0, .9}}));
  auto data = grid.data(); AStar::Cell c; ASSERT_TRUE(grid.worldToCell(.6, 0, c)); data[grid.index(c)] = 1;
  AStar::Grid blocked(grid.width(), grid.height(), grid.resolution(), grid.originX(), grid.originY(), data);
  EXPECT_FALSE(core.collisionFree({{0, 0, 0, 0, 0}}, blocked, {}));
}
TEST(Dwa, NoFeasibleCandidateReturnsZeroInsteadOfOldVelocity) {
  navigation::Parameters p; navigation::DwaCore core(p, footprint());
  const auto raw = emptyGrid(), inflated = raw.inflated(1);
  const auto field = AStar::Generator(inflated).distanceField({35, 20});
  const auto result = core.plan({0, 0, 0, .5, 0}, {7, 0, 0, 0, 0}, raw, inflated, field, {{.9, 0}}, .1);
  EXPECT_FALSE(result.valid); EXPECT_DOUBLE_EQ(result.velocity, 0); EXPECT_DOUBLE_EQ(result.yawrate, 0);
}
TEST(Dwa, AcceleratesTowardGoalAndHonoursPredictionTime) {
  navigation::Parameters p; p.predict_time = 2.0;
  navigation::DwaCore core(p, footprint()); const auto raw = emptyGrid(), inflated = raw.inflated(1);
  const auto field = AStar::Generator(inflated).distanceField({35, 20});
  const auto result = core.plan({0, 0, 0, 0, 0}, {7, 0, 0, 0, 0}, raw, inflated, field, {}, .1);
  ASSERT_TRUE(result.valid); EXPECT_GT(result.velocity, 0); EXPECT_LE(result.velocity, .05 + 1e-8);
  const auto straight = core.simulate({0, 0, 0, 0, 0}, 1, 0);
  EXPECT_NEAR(straight.back().x, 2, 1e-8); EXPECT_NEAR(straight.back().y, 0, 1e-8);
  EXPECT_LE(straight.size(), 22u);
}
TEST(Dwa, GoalBearingCostDoesNotUseFinalGoalYaw) {
  navigation::Parameters p;
  p.goal_bearing_gain = 3;
  const auto raw = emptyGrid(), inflated = raw.inflated(1);
  AStar::Cell cell;
  ASSERT_TRUE(inflated.worldToCell(0, 7, cell));
  const auto field = AStar::Generator(inflated).distanceField(cell);
  const navigation::State start{0, 0, 0, 0, 0};
  const navigation::DwaCore core(p, footprint());
  const auto first = core.plan(start, {0, 7, 0, 0, 0}, raw, inflated, field, {}, .1);
  const auto rotated = core.plan(start, {0, 7, 2.4, 0, 0}, raw, inflated, field, {}, .1);
  ASSERT_TRUE(first.valid); ASSERT_TRUE(rotated.valid);
  EXPECT_DOUBLE_EQ(first.velocity, rotated.velocity);
  EXPECT_DOUBLE_EQ(first.yawrate, rotated.yawrate);
  EXPECT_DOUBLE_EQ(first.score, rotated.score);
  p.goal_bearing_gain = 0;
  const auto unweighted = navigation::DwaCore(p, footprint()).plan(
      start, {0, 7, 0, 0, 0}, raw, inflated, field, {}, .1);
  ASSERT_TRUE(unweighted.valid);
  EXPECT_GT(first.score, unweighted.score + .1);
}

TEST(Dwa, PreferredVelocityIsNotASpeedLimitAndCanBeWeighted) {
  navigation::Parameters p;
  ASSERT_DOUBLE_EQ(p.preferred_velocity, .8);
  ASSERT_DOUBLE_EQ(p.max_velocity, 1);
  ASSERT_DOUBLE_EQ(p.speed_gain, 0);
  const auto raw = emptyGrid(), inflated = raw.inflated(1);
  AStar::Cell cell;
  ASSERT_TRUE(inflated.worldToCell(7, 0, cell));
  const auto field = AStar::Generator(inflated).distanceField(cell);
  const navigation::State start{0, 0, 0, .8, 0}, goal{7, 0, 0, 0, 0};
  const auto unweighted = navigation::DwaCore(p, footprint()).plan(
      start, goal, raw, inflated, field, {}, .1);
  ASSERT_TRUE(unweighted.valid);
  EXPECT_GT(unweighted.velocity, p.preferred_velocity);
  EXPECT_LE(unweighted.velocity, p.max_velocity);
  p.speed_gain = 1000;
  const auto weighted = navigation::DwaCore(p, footprint()).plan(
      start, goal, raw, inflated, field, {}, .1);
  ASSERT_TRUE(weighted.valid);
  EXPECT_NEAR(weighted.velocity, p.preferred_velocity, 1e-9);
}

TEST(Dwa, InvalidSettingsOrInputCannotGenerateCommands) {
  navigation::Parameters p; p.simulation_dt = 1;
  EXPECT_THROW(navigation::DwaCore(p, footprint()), std::invalid_argument);
  p.simulation_dt = .1; navigation::DwaCore core(p, footprint());
  const auto raw = emptyGrid();
  const auto result = core.plan({}, {}, raw, raw, {}, {}, .1);
  EXPECT_FALSE(result.valid); EXPECT_DOUBLE_EQ(result.velocity, 0);
}

TEST(Dwa, OutsideLimitMeasurementsKeepLegalRecoveryCommands) {
  navigation::Parameters p; navigation::DwaCore core(p, footprint());
  const auto raw = emptyGrid(), inflated = raw.inflated(1);
  const auto field = AStar::Generator(inflated).distanceField({35, 20});
  const navigation::State goal{7, 0, 0, 0, 0};
  // A small reverse drift must retain a zero-speed feedback target instead of
  // declaring the unobstructed scene blocked and bypassing the PID entirely.
  const navigation::State reverse_drift{0, 0, 0, -.06, 0};
  auto result = core.plan(reverse_drift, goal, raw, inflated, field, {}, .1);
  ASSERT_TRUE(result.valid); EXPECT_DOUBLE_EQ(result.velocity, 0);
  EXPECT_DOUBLE_EQ(reverse_drift.velocity, -.06);
  const navigation::State overspeed{0, 0, 0, 1.06, 0};
  result = core.plan(overspeed, goal, raw, inflated, field, {}, .1);
  ASSERT_TRUE(result.valid); EXPECT_DOUBLE_EQ(result.velocity, p.max_velocity);
  EXPECT_LT(result.velocity, overspeed.velocity);
  for (const double observed : {-.56, .56}) {
    const navigation::State turn{0, 0, 0, 0, observed};
    result = core.plan(turn, goal, raw, inflated, field, {}, .1);
    ASSERT_TRUE(result.valid);
    EXPECT_DOUBLE_EQ(result.yawrate, std::copysign(p.max_yawrate, observed));
    EXPECT_LT(std::abs(result.yawrate), std::abs(turn.yawrate));
    EXPECT_GE(result.velocity, p.min_velocity);
    EXPECT_LE(result.velocity, p.max_velocity);
  }
}

TEST(Recovery, ReorientsBackwardStationaryBoatUsingTerminalFieldOnly) {
  navigation::Parameters parameters;
  EXPECT_DOUBLE_EQ(parameters.speed_gain, 0);
  EXPECT_DOUBLE_EQ(parameters.goal_bearing_gain, 0);
  navigation::DwaCore core(parameters, footprint());
  const auto raw = emptyGrid(80, 80, .25), inflated = raw.inflated(1);
  AStar::Cell goal_cell;
  ASSERT_TRUE(inflated.worldToCell(7, 0, goal_cell));
  const auto field = AStar::Generator(inflated).distanceField(goal_cell);
  navigation::State state{0, 0, 3.14159265358979323846, 0, 0};
  double heading = 0;
  ASSERT_TRUE(core.descentHeading(state, inflated, field, heading));
  EXPECT_LT(std::abs(navigation::DwaCore::normalizeAngle(heading)), .2);
  bool completed = false;
  for (int i = 0; i < 150; ++i) {
    if (std::abs(navigation::DwaCore::normalizeAngle(heading - state.yaw)) < .2) {
      completed = true; break;
    }
    const auto turn = core.turnInPlace(state, heading, raw, {}, .1);
    ASSERT_TRUE(turn.valid);
    EXPECT_DOUBLE_EQ(turn.velocity, 0);
    EXPECT_LE(std::abs(turn.yawrate - state.yawrate), .05 + 1e-9);
    EXPECT_LE(std::abs(turn.yawrate), parameters.max_yawrate);
    state = core.simulate(state, 0, turn.yawrate, .1).back();
    ASSERT_TRUE(core.collisionFree({state}, raw, {}));
  }
  ASSERT_TRUE(completed);
  EXPECT_NEAR(state.x, 0, 1e-12); EXPECT_NEAR(state.y, 0, 1e-12);
  const auto forward = core.plan(state, {7, 0, 0, 0, 0}, raw, inflated, field, {}, .1);
  ASSERT_TRUE(forward.valid); EXPECT_GT(forward.velocity, 0);
}

TEST(Recovery, RejectsFullHullRotationNearWallAndHonoursOpposingYawAcceleration) {
  navigation::Parameters parameters;
  const std::vector<navigation::Point> long_hull{{-2, -.5}, {2, -.5}, {2, .5}, {-2, .5}};
  navigation::DwaCore core(parameters, long_hull);
  const auto raw = emptyGrid();
  const navigation::State stationary{0, 0, 0, 0, 0};
  ASSERT_TRUE(core.collisionFree({stationary}, raw, {{0, 1.1}}));
  // A short initial 0.05 rad/s sample would look safe, but the complete swept
  // footprint cannot turn north without hitting the observed wall.
  const auto blocked = core.turnInPlace(stationary, 1.57079632679, raw, {{0, 1.1}}, .1);
  EXPECT_FALSE(blocked.valid); EXPECT_DOUBLE_EQ(blocked.yawrate, 0);
  const navigation::State opposing_yaw{0, 0, 0, 0, -.2};
  const auto braking = core.turnInPlace(opposing_yaw, 1.57079632679, raw, {}, .1);
  ASSERT_TRUE(braking.valid); EXPECT_NEAR(braking.yawrate, -.15, 1e-12);
  EXPECT_FALSE(core.turnInPlace({0, 0, 0, .2, 0}, 1.5, raw, {}, .1).valid);
}

TEST(Recovery, CannotSelectDirectionFromInvalidUnreachableOrFlatField) {
  navigation::DwaCore core(navigation::Parameters{}, footprint());
  const auto raw = emptyGrid();
  double heading = 123;
  EXPECT_FALSE(core.descentHeading({}, raw, {}, heading));
  EXPECT_FALSE(core.descentHeading({}, raw, std::vector<double>(raw.size(), 5), heading));
  EXPECT_FALSE(core.descentHeading({}, raw, std::vector<double>(raw.size(),
      std::numeric_limits<double>::infinity()), heading));
  EXPECT_DOUBLE_EQ(heading, 123);
}

TEST(Observations, UnknownBlockChangesRoutingWithoutChangingSurveyedMap) {
  const auto known = emptyGrid(30, 20, 1, -15, -10);
  navigation::ObservationLayer layer; layer.reset(known.size());
  std::vector<navigation::Point> block;
  for (double y = -.5; y <= 1.5; y += .5) {
    block.push_back({-.5, y}); block.push_back({1.5, y});
  }
  for (double x = -.5; x <= 1.5; x += .5) {
    block.push_back({x, -.5}); block.push_back({x, 1.5});
  }
  layer.observe(known, block, 10.0);
  const auto live = layer.combined(known, 11.0, 5.0).inflated(2);
  const AStar::Cell start{8, 10}, goal{23, 10};
  const auto direct = AStar::Generator(known.inflated(2)).findPath(start, goal);
  const auto detour = AStar::Generator(live).findPath(start, goal);
  ASSERT_TRUE(direct.found); ASSERT_TRUE(detour.found);
  EXPECT_GT(detour.cost, direct.cost + 1);
  EXPECT_FALSE(known.occupied({15, 10}));
  EXPECT_TRUE(live.occupied({15, 10}));
}

TEST(Observations, DynamicReturnsExpireAndKnownWallNoiseDoesNotGrowTheMap) {
  auto data = std::vector<std::uint8_t>(100, 0); data[55] = 1;
  const AStar::Grid known(10, 10, .5, -2.5, -2.5, data);
  navigation::ObservationLayer layer; layer.reset(known.size());
  layer.observe(known, {{.75, .25}, {-1.25, -1.25}}, 10, .5);
  const auto before = layer.combined(known, 14, 5);
  EXPECT_FALSE(before.occupied({6, 5}));  // One-cell noise beside the known wall.
  EXPECT_TRUE(before.occupied({2, 2}));
  EXPECT_EQ(layer.combined(known, 15.01, 5).data(), known.data());
  layer.reset(known.size());
  EXPECT_EQ(layer.combined(known, 2, 5).data(), known.data());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
