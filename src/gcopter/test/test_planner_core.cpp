#include "gcopter/corridor_generator.hpp"
#include "gcopter/local_replan.hpp"
#include "gcopter/path_search.hpp"
#include "gcopter/trajectory_generation.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

namespace
{
GridMap2D makeMap(int width = 20, int height = 20, double resolution = 0.1)
{
  GridMap2D map;
  map.frame_id = "map";
  map.resolution = resolution;
  map.width = width;
  map.height = height;
  map.raw_occupancy.assign(map.cellCount(), 0U);
  map.semantic.assign(map.cellCount(), static_cast<uint8_t>(MapSemantic::NORMAL));
  EXPECT_TRUE(map.buildDerivedLayers(0.0, 0.0));
  return map;
}

void rebuild(GridMap2D & map) {ASSERT_TRUE(map.buildDerivedLayers(0.0, 0.0));}

PathSearch::Options searchOptions()
{
  PathSearch::Options options;
  options.turn_cost_weight = 0.2;
  options.safety_cost_distance = 0.3;
  return options;
}
}  // namespace

TEST(PathSearch, OpenSpaceUsesDiagonalAndPreservesEndpoints)
{
  const GridMap2D map = makeMap();
  const Eigen::Vector2d start(0.15, 0.15);
  const Eigen::Vector2d goal(1.55, 1.55);
  std::vector<Eigen::Vector2d> path;
  ASSERT_TRUE(PathSearch().search(map, start, goal, true, searchOptions(), path));
  EXPECT_TRUE(path.front().isApprox(start));
  EXPECT_TRUE(path.back().isApprox(goal));
  EXPECT_LT(path.size(), 18U);
}

TEST(PathSearch, DetoursAroundWall)
{
  GridMap2D map = makeMap();
  for (int y = 0; y < 16; ++y) {
    if (y != 8) {map.raw_occupancy[static_cast<size_t>(map.index({10, y}))] = 100U;}
  }
  rebuild(map);
  std::vector<Eigen::Vector2d> path;
  ASSERT_TRUE(PathSearch().search(map, {0.25, 0.25}, {1.75, 0.25}, true,
    searchOptions(), path));
  EXPECT_TRUE(std::any_of(path.begin(), path.end(), [](const Eigen::Vector2d & point) {
    return point.y() > 0.75;
  }));
}

TEST(PathSearch, ReportsTemporarilyOccupiedEndpoint)
{
  GridMap2D map = makeMap();
  const Eigen::Vector2d start = map.gridToWorld({2, 2});
  map.raw_occupancy[static_cast<size_t>(map.index({2, 2}))] = 100U;
  rebuild(map);
  std::vector<Eigen::Vector2d> path;
  std::string reason;
  EXPECT_FALSE(PathSearch().search(
    map, start, map.gridToWorld({15, 15}), true, searchOptions(), path, &reason));
  EXPECT_EQ(reason, "start_occupied");
}

TEST(PathSearch, CrossHolePenaltySelectsNormalAlternative)
{
  GridMap2D map = makeMap(20, 10, 0.1);
  for (int x = 7; x <= 12; ++x) {
    map.semantic[static_cast<size_t>(map.index({x, 5}))] =
      static_cast<uint8_t>(MapSemantic::CROSS_HOLE);
  }
  rebuild(map);
  PathSearch::Options options = searchOptions();
  options.special_region_cost_weight = 20.0;
  std::vector<Eigen::Vector2d> path;
  ASSERT_TRUE(PathSearch().search(map, {0.25, 0.55}, {1.75, 0.55}, true, options, path));
  EXPECT_FALSE(std::any_of(path.begin(), path.end(), [&](const Eigen::Vector2d & point) {
    return map.semanticAt(map.worldToGrid(point)) == MapSemantic::CROSS_HOLE;
  }));
}

TEST(PathSearch, ReusesPreparedStateWithoutLeakingPreviousSearch)
{
  GridMap2D map = makeMap(30, 20, 0.1);
  for (int y = 2; y < 18; ++y) {
    if (y != 10) {map.raw_occupancy[static_cast<size_t>(map.index({15, y}))] = 100U;}
  }
  rebuild(map);
  PathSearch search;
  search.prepare(map.cellCount());
  std::vector<Eigen::Vector2d> first;
  std::vector<Eigen::Vector2d> second;
  ASSERT_TRUE(search.search(
    map, map.gridToWorld({2, 3}), map.gridToWorld({27, 16}), true,
    searchOptions(), first));
  ASSERT_TRUE(search.search(
    map, map.gridToWorld({2, 16}), map.gridToWorld({27, 3}), true,
    searchOptions(), second));
  EXPECT_TRUE(first.front().isApprox(map.gridToWorld({2, 3})));
  EXPECT_TRUE(first.back().isApprox(map.gridToWorld({27, 16})));
  EXPECT_TRUE(second.front().isApprox(map.gridToWorld({2, 16})));
  EXPECT_TRUE(second.back().isApprox(map.gridToWorld({27, 3})));
}

TEST(SamplePath, PreservesTurnsAndSemanticBoundaries)
{
  GridMap2D map = makeMap(10, 10, 0.1);
  map.semantic[static_cast<size_t>(map.index({3, 1}))] =
    static_cast<uint8_t>(MapSemantic::CROSS_HOLE);
  rebuild(map);
  std::vector<Eigen::Vector2d> input;
  for (int x = 1; x <= 5; ++x) {input.push_back(map.gridToWorld({x, 1}));}
  for (int y = 2; y <= 5; ++y) {input.push_back(map.gridToWorld({5, y}));}
  std::vector<Eigen::Vector2d> output;
  ASSERT_TRUE(SamplePath(map, input, output));
  EXPECT_LT(output.size(), input.size());
  EXPECT_TRUE(std::any_of(output.begin(), output.end(), [&](const Eigen::Vector2d & point) {
    return map.semanticAt(map.worldToGrid(point)) == MapSemantic::CROSS_HOLE;
  }));
  EXPECT_TRUE(std::any_of(output.begin(), output.end(), [&](const Eigen::Vector2d & point) {
    return map.worldToGrid(point) == Eigen::Vector2i(5, 1);
  }));
}

TEST(SamplePath, BoundsLongStraightSeedSpacing)
{
  GridMap2D map = makeMap(50, 10, 0.1);
  std::vector<Eigen::Vector2d> input;
  for (int x = 1; x <= 41; ++x) {input.push_back(map.gridToWorld({x, 2}));}
  std::vector<Eigen::Vector2d> output;
  ASSERT_TRUE(SamplePath(map, input, output));
  ASSERT_GT(output.size(), 2U);
  for (size_t index = 1; index < output.size(); ++index) {
    const Eigen::Vector2i previous = map.worldToGrid(output[index - 1]);
    const Eigen::Vector2i current = map.worldToGrid(output[index]);
    EXPECT_LE((current - previous).cwiseAbs().maxCoeff(), 10);
  }
}

TEST(Corridor, HasAreaOverlapAtNinetyDegreeTurn)
{
  const GridMap2D map = makeMap(40, 40, 0.1);
  const std::vector<Eigen::Vector2d> path{{0.5, 0.5}, {2.0, 0.5}, {2.0, 2.0}};
  CorridorOptions options;
  options.minimum_overlap = 0.08;
  std::vector<ConvexCorridor2D> corridors;
  std::string reason;
  ASSERT_TRUE(GenerateCorridor(map, path, options, corridors, reason)) << reason;
  ASSERT_EQ(corridors.size(), 2U);
  EXPECT_TRUE(corridors[0].contains({2.0, 0.54}));
  EXPECT_TRUE(corridors[1].contains({2.0, 0.54}));
}

TEST(Corridor, AcceptsAsymmetricAreaOverlapAtBlockedTurn)
{
  GridMap2D map = makeMap(30, 30, 0.1);
  // Obstacles immediately beyond the incoming segment and behind the outgoing
  // segment force the overlap into just the free upper-left quadrant.
  map.raw_occupancy[static_cast<size_t>(map.index({11, 5}))] = 100U;
  map.raw_occupancy[static_cast<size_t>(map.index({10, 4}))] = 100U;
  rebuild(map);
  const std::vector<Eigen::Vector2d> path{{0.55, 0.55}, {1.05, 0.55}, {1.05, 1.05}};
  CorridorOptions options;
  options.minimum_overlap = 0.12;
  std::vector<ConvexCorridor2D> corridors;
  std::string reason;
  ASSERT_TRUE(GenerateCorridor(map, path, options, corridors, reason)) << reason;
  ASSERT_EQ(corridors.size(), 2U);
  EXPECT_TRUE(corridors[0].contains({1.04, 0.56}));
  EXPECT_TRUE(corridors[1].contains({1.04, 0.56}));
}

TEST(Trajectory, PreservesNonZeroReplanBoundaryState)
{
  const GridMap2D map = makeMap(40, 20, 0.1);
  const std::vector<Eigen::Vector2d> path{{0.5, 0.5}, {1.5, 0.5}, {2.5, 0.5}};
  std::vector<ConvexCorridor2D> corridors;
  std::string reason;
  ASSERT_TRUE(GenerateCorridor(map, path, CorridorOptions(), corridors, reason)) << reason;
  Eigen::Matrix<double, 2, 3> start_pva = Eigen::Matrix<double, 2, 3>::Zero();
  start_pva.col(1) = Eigen::Vector2d(0.3, 0.0);
  MincoTrajectoryData data;
  ASSERT_TRUE(GenerateTrajectory(path, corridors, 0.5, 0.1, start_pva, data));
  EXPECT_TRUE(data.start_pva.col(1).isApprox(Eigen::Vector2d(0.3, 0.0)));
  EXPECT_GE(data.segment_durations.minCoeff(), 0.1);
}

TEST(Trajectory, MincoOptimizationRespectsPlanningEnvelope)
{
  const GridMap2D map = makeMap(50, 30, 0.1);
  const std::vector<Eigen::Vector2d> path{{0.5, 0.8}, {1.5, 0.8}, {2.5, 1.2}};
  std::vector<ConvexCorridor2D> corridors;
  std::string reason;
  ASSERT_TRUE(GenerateCorridor(map, path, CorridorOptions(), corridors, reason)) << reason;
  MincoTrajectoryData initial;
  const Eigen::Matrix<double, 2, 3> zero_pva = Eigen::Matrix<double, 2, 3>::Zero();
  ASSERT_TRUE(GenerateTrajectory(path, corridors, 0.8, 0.1, zero_pva, initial));
  OptimizationOptions options;
  options.max_velocity = 0.7;
  options.max_acceleration = 1.0;
  options.time_budget_ms = 100.0;
  MincoTrajectoryData optimized;
  ASSERT_TRUE(OptimizeTrajectory(map, initial, options, optimized));
  EXPECT_EQ(optimized.segment_count, 2);
  EXPECT_TRUE(optimized.segment_durations.allFinite());
  EXPECT_TRUE((optimized.segment_durations.array() >= 0.1).all());
}

TEST(Trajectory, SafeInitialFallbackUsesTimeScalingWithoutMovingWaypoints)
{
  const GridMap2D map = makeMap(60, 30, 0.1);
  const std::vector<Eigen::Vector2d> path{{0.5, 0.8}, {1.5, 0.8}, {2.5, 1.2}};
  std::vector<ConvexCorridor2D> corridors;
  std::string reason;
  ASSERT_TRUE(GenerateCorridor(map, path, CorridorOptions(), corridors, reason)) << reason;
  MincoTrajectoryData initial;
  const Eigen::Matrix<double, 2, 3> zero_pva = Eigen::Matrix<double, 2, 3>::Zero();
  ASSERT_TRUE(GenerateTrajectory(path, corridors, 2.0, 0.05, zero_pva, initial));

  OptimizationOptions options;
  options.max_velocity = 0.35;
  options.max_acceleration = 0.45;
  MincoTrajectoryData fallback;
  ASSERT_TRUE(PrepareSafeInitialTrajectory(map, initial, options, fallback, &reason)) << reason;
  EXPECT_TRUE(fallback.intermediate_positions.isApprox(initial.intermediate_positions));
  EXPECT_TRUE((fallback.segment_durations.array() >= initial.segment_durations.array()).all());
}

TEST(LocalReplan, RejoinsOriginalSuffixWithinAbsoluteDistanceLimits)
{
  GridMap2D map = makeMap(70, 30, 0.1);
  const std::vector<Eigen::Vector2d> active_route{
    {0.5, 1.0}, {1.0, 1.0}, {1.5, 1.0}, {2.0, 1.0}, {2.5, 1.0},
    {3.0, 1.0}, {3.5, 1.0}, {4.0, 1.0}, {4.5, 1.0}};
  const Eigen::Vector2i blocked = map.worldToGrid({1.5, 1.0});
  map.raw_occupancy[static_cast<size_t>(map.index(blocked))] = 100U;
  rebuild(map);

  LocalReplanOptions options;
  options.minimum_lookahead_distance = 1.0;
  options.maximum_lookahead_distance = 3.0;
  options.maximum_local_path_distance = 5.0;
  options.maximum_extra_distance = 1.0;
  std::vector<Eigen::Vector2d> output;
  size_t rejoin_index = 0;
  std::string reason;
  ASSERT_TRUE(BuildLocalRejoinRoute(
    map, active_route, active_route.front(), true, searchOptions(), options,
    output, rejoin_index, reason)) << reason;
  EXPECT_TRUE(output.front().isApprox(active_route.front()));
  EXPECT_TRUE(output.back().isApprox(active_route.back()));
  ASSERT_LT(rejoin_index + 1, active_route.size());
  EXPECT_TRUE(std::find_if(output.begin(), output.end(), [&](const Eigen::Vector2d & point) {
    return point.isApprox(active_route[rejoin_index + 1]);
  }) != output.end());
}

TEST(LocalReplan, RejectsDetourBeyondAbsoluteExtraDistance)
{
  GridMap2D map = makeMap(70, 30, 0.1);
  const std::vector<Eigen::Vector2d> active_route{
    {0.5, 1.0}, {1.0, 1.0}, {1.5, 1.0}, {2.0, 1.0}, {2.5, 1.0}, {3.0, 1.0}};
  const Eigen::Vector2i blocked = map.worldToGrid({1.5, 1.0});
  map.raw_occupancy[static_cast<size_t>(map.index(blocked))] = 100U;
  rebuild(map);
  LocalReplanOptions options;
  options.minimum_lookahead_distance = 1.0;
  options.maximum_lookahead_distance = 2.5;
  options.maximum_local_path_distance = 5.0;
  options.maximum_extra_distance = 0.0;
  std::vector<Eigen::Vector2d> output;
  size_t rejoin_index = 0;
  std::string reason;
  EXPECT_FALSE(BuildLocalRejoinRoute(
    map, active_route, active_route.front(), true, searchOptions(), options,
    output, rejoin_index, reason));
  EXPECT_EQ(reason, "no_rejoin_within_absolute_limits");
}
