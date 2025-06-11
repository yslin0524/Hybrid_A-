#pragma once

#include <memory>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "hybrid_a_star_planner/hybrid_a_star.hpp"

namespace hybrid_a_star_planner {

bool tryShotToGoal(
    const NodePtr& current,
    const geometry_msgs::msg::Pose& goal,
    const nav_msgs::msg::OccupancyGrid& map,
    std::vector<geometry_msgs::msg::PoseStamped>& path_segment);

}
