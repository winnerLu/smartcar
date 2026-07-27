/**
 * Copyright 2025 Suchetan Saravanan.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#ifndef ROADMAP_EXPLORER__FRONTIER_GOAL_SANITIZER_HPP_
#define ROADMAP_EXPLORER__FRONTIER_GOAL_SANITIZER_HPP_

#include <cmath>
#include <limits>
#include <queue>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

namespace roadmap_explorer
{

struct FrontierGoalProjection
{
  bool valid{false};
  bool adjusted{false};
  geometry_msgs::msg::Point point;
  double projection_distance{std::numeric_limits<double>::infinity()};
};

/**
 * @brief Project a frontier reference onto the latest reachable known-free map.
 *
 * Roadmap Explorer represents a frontier with an unknown grid cell. Sending
 * that cell directly to Nav2 is unsafe when it lies on the exclusive upper map
 * boundary or when Nav2 has not consumed the same map resize yet. This search
 * starts at the robot and visits only known traversable cells, so the selected
 * result is connected to the robot in the current exploration costmap.
 *
 * Candidate cells must also remain boundary_margin metres inside every map
 * edge. If no reachable candidate is close enough to the frontier, the
 * frontier is rejected instead of dispatching an out-of-bounds goal.
 */
inline FrontierGoalProjection projectFrontierGoalToKnownFree(
  const nav2_costmap_2d::Costmap2D & costmap,
  const geometry_msgs::msg::Point & frontier,
  const geometry_msgs::msg::Point & robot,
  double boundary_margin,
  double max_projection_distance,
  double robot_seed_search_radius = 0.30)
{
  FrontierGoalProjection result;
  const auto size_x = costmap.getSizeInCellsX();
  const auto size_y = costmap.getSizeInCellsY();
  const double resolution = costmap.getResolution();
  if (size_x == 0 || size_y == 0 || resolution <= 0.0) {
    return result;
  }

  const unsigned int margin_cells = static_cast<unsigned int>(
    std::max(0.0, std::ceil(boundary_margin / resolution)));
  if (
    size_x <= (2 * margin_cells) ||
    size_y <= (2 * margin_cells))
  {
    return result;
  }

  unsigned int robot_x = 0;
  unsigned int robot_y = 0;
  if (!costmap.worldToMap(robot.x, robot.y, robot_x, robot_y)) {
    return result;
  }

  const auto is_traversable =
    [&costmap](unsigned int x, unsigned int y)
    {
      const auto cost = costmap.getCost(x, y);
      return cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
             cost != nav2_costmap_2d::NO_INFORMATION;
    };
  unsigned int seed_x = robot_x;
  unsigned int seed_y = robot_y;
  if (!is_traversable(seed_x, seed_y)) {
    // The exploration costmap does not clear the robot footprint. Close to an
    // inflated obstacle the cell containing base_link can therefore be
    // INSCRIBED even though Nav2 still has a valid path away from it. Starting
    // strictly at that one cell caused every frontier to be rejected in
    // sequence. Seed the connectivity search from the nearest traversable
    // cell around the robot instead.
    const int seed_radius_cells = static_cast<int>(
      std::max(1.0, std::ceil(robot_seed_search_radius / resolution)));
    double nearest_seed_distance_sq = std::numeric_limits<double>::infinity();
    bool found_seed = false;
    for (int offset_y = -seed_radius_cells; offset_y <= seed_radius_cells; ++offset_y) {
      for (int offset_x = -seed_radius_cells; offset_x <= seed_radius_cells; ++offset_x) {
        const int candidate_x = static_cast<int>(robot_x) + offset_x;
        const int candidate_y = static_cast<int>(robot_y) + offset_y;
        if (
          candidate_x < 0 || candidate_y < 0 ||
          candidate_x >= static_cast<int>(size_x) ||
          candidate_y >= static_cast<int>(size_y))
        {
          continue;
        }
        const double distance_sq =
          static_cast<double>((offset_x * offset_x) + (offset_y * offset_y));
        if (
          distance_sq > static_cast<double>(seed_radius_cells * seed_radius_cells) ||
          distance_sq >= nearest_seed_distance_sq)
        {
          continue;
        }
        const auto unsigned_x = static_cast<unsigned int>(candidate_x);
        const auto unsigned_y = static_cast<unsigned int>(candidate_y);
        if (!is_traversable(unsigned_x, unsigned_y)) {
          continue;
        }
        seed_x = unsigned_x;
        seed_y = unsigned_y;
        nearest_seed_distance_sq = distance_sq;
        found_seed = true;
      }
    }
    if (!found_seed) {
      return result;
    }
  }

  const auto is_interior =
    [size_x, size_y, margin_cells](unsigned int x, unsigned int y)
    {
      return x >= margin_cells && y >= margin_cells &&
             x < (size_x - margin_cells) &&
             y < (size_y - margin_cells);
    };

  const auto index =
    [size_x](unsigned int x, unsigned int y)
    {
      return static_cast<size_t>(y) * size_x + x;
    };

  std::vector<bool> visited(static_cast<size_t>(size_x) * size_y, false);
  std::queue<std::pair<unsigned int, unsigned int>> pending;
  pending.emplace(seed_x, seed_y);
  visited[index(seed_x, seed_y)] = true;

  double best_distance_sq = std::numeric_limits<double>::infinity();
  geometry_msgs::msg::Point best_point;
  constexpr int dx[4] = {1, -1, 0, 0};
  constexpr int dy[4] = {0, 0, 1, -1};

  while (!pending.empty()) {
    const auto [x, y] = pending.front();
    pending.pop();

    if (is_interior(x, y)) {
      double wx = 0.0;
      double wy = 0.0;
      costmap.mapToWorld(x, y, wx, wy);
      const double distance_sq =
        std::pow(wx - frontier.x, 2) + std::pow(wy - frontier.y, 2);
      if (distance_sq < best_distance_sq) {
        best_distance_sq = distance_sq;
        best_point.x = wx;
        best_point.y = wy;
        best_point.z = frontier.z;
      }
    }

    for (size_t direction = 0; direction < 4; ++direction) {
      const int next_x = static_cast<int>(x) + dx[direction];
      const int next_y = static_cast<int>(y) + dy[direction];
      if (
        next_x < 0 || next_y < 0 ||
        next_x >= static_cast<int>(size_x) ||
        next_y >= static_cast<int>(size_y))
      {
        continue;
      }
      const auto unsigned_x = static_cast<unsigned int>(next_x);
      const auto unsigned_y = static_cast<unsigned int>(next_y);
      const auto next_index = index(unsigned_x, unsigned_y);
      if (visited[next_index] || !is_traversable(unsigned_x, unsigned_y)) {
        continue;
      }
      visited[next_index] = true;
      pending.emplace(unsigned_x, unsigned_y);
    }
  }

  if (!std::isfinite(best_distance_sq)) {
    return result;
  }
  result.projection_distance = std::sqrt(best_distance_sq);
  if (result.projection_distance > max_projection_distance) {
    return result;
  }

  result.valid = true;
  result.point = best_point;
  result.adjusted = result.projection_distance > (resolution * 0.25);
  return result;
}

}  // namespace roadmap_explorer

#endif  // ROADMAP_EXPLORER__FRONTIER_GOAL_SANITIZER_HPP_
