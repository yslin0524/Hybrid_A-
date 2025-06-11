#include "hybrid_a_star_planner/hybrid_a_star.hpp"
#include "hybrid_a_star_planner/shot_to_goal.hpp"
#include "hybrid_a_star_planner/smoother.hpp"
#include "hybrid_a_star_planner/utils/planner_utils.hpp"
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>

namespace hybrid_a_star_planner {

bool HybridAStar::tryShotToGoal(
    const NodePtr& current,
    const geometry_msgs::msg::Pose& goal,
    const nav_msgs::msg::OccupancyGrid& map,
    std::vector<geometry_msgs::msg::PoseStamped>& out_segment)
{
    // Angular tolerance: 5 degrees
    const double theta_tol = 5.0 * M_PI / 180.0;

    // 1) Attempt direct Reeds–Shepp from current->real_pose to goal
    auto shot = tryReedsSheppMove(
        current->real_pose,
        goal,
        map,
        map.info.resolution,
        allow_unknown_,
        obstacle_threshold_);
    if (!shot) {
        return false;
    }

    // 2) Collision check along the path segment
    if (!isPathCollisionFree(shot->path_segment,
                             map,
                             obstacle_threshold_,
                             allow_unknown_))
    {
        return false;
    }

    // 3) Angle check at the end
    double final_yaw = tf2::getYaw(shot->end_pose.pose.orientation);
    double goal_yaw  = tf2::getYaw(goal.orientation);
    if (std::fabs(normalizeAngle(goal_yaw - final_yaw)) > theta_tol) {
        return false;
    }

    // 4) Stamp headers and output
    for (auto& ps : shot->path_segment) {
        ps.header = map.header;
    }
    out_segment = std::move(shot->path_segment);
    return true;
}

}  // namespace hybrid_a_star_planner