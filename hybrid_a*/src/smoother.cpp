#include "hybrid_a_star_planner/smoother.hpp"
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "hybrid_a_star_planner/utils/planner_utils.hpp"

namespace hybrid_a_star_planner {

std::optional<RSExpansionResult> tryReedsSheppMove(
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::Pose& goal,
    const nav_msgs::msg::OccupancyGrid& occupancy_grid,
    double resolution,
    bool allow_unknown,
    int obstacle_threshold) {

  ompl::base::ReedsSheppStateSpace state_space(3.0);  // turning radius

  ompl::base::State* start_state = state_space.allocState();
  ompl::base::State* goal_state = state_space.allocState();

  start_state->as<ompl::base::SE2StateSpace::StateType>()->setXY(start.position.x, start.position.y);
  start_state->as<ompl::base::SE2StateSpace::StateType>()->setYaw(tf2::getYaw(start.orientation));

  goal_state->as<ompl::base::SE2StateSpace::StateType>()->setXY(goal.position.x, goal.position.y);
  goal_state->as<ompl::base::SE2StateSpace::StateType>()->setYaw(tf2::getYaw(goal.orientation));

  double total_len = state_space.distance(start_state, goal_state);
  int steps = std::max(2, static_cast<int>(total_len / resolution * 2));

  std::vector<geometry_msgs::msg::PoseStamped> segment;

  int width = occupancy_grid.info.width;
  int height = occupancy_grid.info.height;
  double res = occupancy_grid.info.resolution;

  for (int i = 0; i <= steps; ++i) {
    double t = static_cast<double>(i) / steps;
    ompl::base::State* interp = state_space.allocState();
    state_space.interpolate(start_state, goal_state, t, interp);

    double x = interp->as<ompl::base::SE2StateSpace::StateType>()->getX();
    double y = interp->as<ompl::base::SE2StateSpace::StateType>()->getY();
    double yaw = interp->as<ompl::base::SE2StateSpace::StateType>()->getYaw();

    int ci = static_cast<int>((x - occupancy_grid.info.origin.position.x) / res);
    int cj = static_cast<int>((y - occupancy_grid.info.origin.position.y) / res);

    if (ci < 0 || ci >= width || cj < 0 || cj >= height ||
      planner_utils::isOccupiedIndex(occupancy_grid, ci + cj * width, allow_unknown, obstacle_threshold)) {
      state_space.freeState(interp);
      state_space.freeState(start_state);
      state_space.freeState(goal_state);
      return std::nullopt;
    }


    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = x;
    p.pose.position.y = y;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    p.pose.orientation = tf2::toMsg(q);

    segment.push_back(p);
    state_space.freeState(interp);
  }

  for (auto& p : segment) {
    p.header = occupancy_grid.header; 
  }

  state_space.freeState(start_state);
  state_space.freeState(goal_state);

  RSExpansionResult result;
  result.path_segment = segment;
  result.end_pose = segment.back();
  return result;
}

}  // namespace hybrid_a_star_planner
