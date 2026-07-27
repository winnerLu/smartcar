#include <gtest/gtest.h>

#include <geometry_msgs/msg/point.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

#include "roadmap_explorer/FrontierGoalSanitizer.hpp"

namespace roadmap_explorer
{

geometry_msgs::msg::Point point(double x, double y)
{
  geometry_msgs::msg::Point output;
  output.x = x;
  output.y = y;
  return output;
}

TEST(FrontierGoalSanitizer, ProjectsUnknownEdgeIntoReachableKnownFreeInterior)
{
  nav2_costmap_2d::Costmap2D costmap(
    20, 20, 0.1, 0.0, 0.0, nav2_costmap_2d::NO_INFORMATION);
  for (unsigned int y = 2; y <= 17; ++y) {
    for (unsigned int x = 2; x <= 17; ++x) {
      costmap.setCost(x, y, nav2_costmap_2d::FREE_SPACE);
    }
  }

  const auto result = projectFrontierGoalToKnownFree(
    costmap, point(1.95, 1.05), point(1.05, 1.05), 0.20, 0.60);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.adjusted);
  EXPECT_LE(result.point.x, 1.75);
  EXPECT_NE(costmap.getCost(
      static_cast<unsigned int>(result.point.x / 0.1),
      static_cast<unsigned int>(result.point.y / 0.1)),
    nav2_costmap_2d::NO_INFORMATION);
}

TEST(FrontierGoalSanitizer, UsesOnlyRobotConnectedFreeSpace)
{
  nav2_costmap_2d::Costmap2D costmap(
    20, 20, 0.1, 0.0, 0.0, nav2_costmap_2d::LETHAL_OBSTACLE);
  for (unsigned int y = 3; y <= 16; ++y) {
    for (unsigned int x = 2; x <= 8; ++x) {
      costmap.setCost(x, y, nav2_costmap_2d::FREE_SPACE);
    }
    for (unsigned int x = 11; x <= 17; ++x) {
      costmap.setCost(x, y, nav2_costmap_2d::FREE_SPACE);
    }
  }

  const auto result = projectFrontierGoalToKnownFree(
    costmap, point(1.25, 1.05), point(0.45, 1.05), 0.20, 1.00);

  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.point.x, 0.90);
}

TEST(FrontierGoalSanitizer, RejectsProjectionBeyondConfiguredLimit)
{
  nav2_costmap_2d::Costmap2D costmap(
    20, 20, 0.1, 0.0, 0.0, nav2_costmap_2d::NO_INFORMATION);
  for (unsigned int y = 4; y <= 8; ++y) {
    for (unsigned int x = 4; x <= 8; ++x) {
      costmap.setCost(x, y, nav2_costmap_2d::FREE_SPACE);
    }
  }

  const auto result = projectFrontierGoalToKnownFree(
    costmap, point(1.85, 1.85), point(0.55, 0.55), 0.20, 0.30);

  EXPECT_FALSE(result.valid);
}

TEST(FrontierGoalSanitizer, RejectsRobotOutsideLatestMap)
{
  nav2_costmap_2d::Costmap2D costmap(
    20, 20, 0.1, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);

  const auto result = projectFrontierGoalToKnownFree(
    costmap, point(1.0, 1.0), point(2.1, 1.0), 0.20, 0.60);

  EXPECT_FALSE(result.valid);
}

TEST(FrontierGoalSanitizer, UsesNearbyFreeSeedWhenRobotCellIsInflated)
{
  nav2_costmap_2d::Costmap2D costmap(
    20, 20, 0.1, 0.0, 0.0, nav2_costmap_2d::NO_INFORMATION);
  for (unsigned int y = 2; y <= 17; ++y) {
    for (unsigned int x = 2; x <= 17; ++x) {
      costmap.setCost(x, y, nav2_costmap_2d::FREE_SPACE);
    }
  }
  costmap.setCost(10, 10, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

  const auto result = projectFrontierGoalToKnownFree(
    costmap, point(1.75, 1.05), point(1.05, 1.05), 0.20, 0.60, 0.30);

  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.projection_distance, 0.30);
}

}  // namespace roadmap_explorer
