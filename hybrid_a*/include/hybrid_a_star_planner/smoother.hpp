// ======= reeds_shepp_expansion.hpp =======
#pragma once

#include <optional>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace hybrid_a_star_planner {

struct RSExpansionResult {
  std::vector<geometry_msgs::msg::PoseStamped> path_segment;
  geometry_msgs::msg::PoseStamped end_pose;
};

// code you add
std::optional<RSExpansionResult> tryReedsSheppMove(
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::Pose& goal,
    const nav_msgs::msg::OccupancyGrid& occupancy_grid,
    double resolution,
    bool allow_unknown,
    int obstacle_threshold);

}  // namespace hybrid_a_star_planner

