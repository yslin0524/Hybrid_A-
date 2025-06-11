#include "hybrid_a_star_planner/hybrid_a_star.hpp"
#include "hybrid_a_star_planner/smoother.hpp"
#include "hybrid_a_star_planner/utils/planner_utils.hpp"
#include "hybrid_a_star_planner/shot_to_goal.hpp"

#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

using namespace std;
using namespace planner_utils;

namespace hybrid_a_star_planner {

HybridAStar::HybridAStar(bool allow_unknown, int obstacle_threshold)
    : allow_unknown_(allow_unknown), obstacle_threshold_(obstacle_threshold) {
    num_angle_bins_ = 72;
    angle_bin_size_ = 2 * PI / num_angle_bins_;
}

nav_msgs::msg::Path HybridAStar::makePlan(
    const nav_msgs::msg::OccupancyGrid& occupancy_grid,
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::Pose& goal) {

    nav_msgs::msg::Path path;
    path.header = occupancy_grid.header;

    int width = occupancy_grid.info.width;
    int height = occupancy_grid.info.height;
    double res = occupancy_grid.info.res;
    double origin_x = occupancy_grid.info.origin.position.x;
    double origin_y = occupancy_grid.info.origin.position.y;
    double origin_yaw = tf2::getYaw(occupancy_grid.info.origin.orientation);
    double cos_ori = cos(origin_yaw);
    double sin_ori = sin(origin_yaw);


    // grid of start/goal
    int start_i, start_j, goal_i, goal_j;

    worldToGrid(start.position.x, start.position.y, origin_x, origin_y, cos_ori, sin_ori, res, start_i, start_j);
    worldToGrid(goal.position.x, goal.position.y, origin_x, origin_y, cos_ori, sin_ori, res, goal_i, goal_j);

    double start_yaw_grid = normalizeAngle(tf2::getYaw(start.orientation) - origin_yaw);
    double goal_yaw_grid = normalizeAngle(tf2::getYaw(goal.orientation) - origin_yaw);




    if (start_i < 0 || start_i >= width || start_j < 0 || start_j >= height ||
        goal_i < 0 || goal_i >= width || goal_j < 0 || goal_j >= height) {
        return path;
    }


    if (isOccupiedIndex(occupancy_grid, start_i + start_j * width, allow_unknown_, obstacle_threshold_) ||
        isOccupiedIndex(occupancy_grid, goal_i + goal_j * width, allow_unknown_, obstacle_threshold_)) {
        return path;
    }


    int start_theta_idx = getAngleBinIndex(start_yaw_grid, num_angle_bins_);
    int goal_theta_idx = getAngleBinIndex(goal_yaw_grid, num_angle_bins_);

    size_t start_idx = getIndex(start_i, start_j, start_theta_idx, width, num_angle_bins_);
    size_t goal_idx = getIndex(goal_i, goal_j, goal_theta_idx, width, num_angle_bins_);

    std::priority_queue<NodePtr, std::vector<NodePtr>, CompareNodes> open;
    std::unordered_map<size_t, double> cost_so_far;

    auto start_node = std::make_shared<Node3D>();
    start_node->real_pose = start;
    start_node->g         = 0.0;
    start_node->f         = std::hypot(
        goal.position.x - start_node->real_pose.position.x,
        goal.position.y - start_node->real_pose.position.y);
    start_node->index     = start_idx; 
    
    geometry_msgs::msg::PoseStamped stamped_start;
    stamped_start.header = path.header;
    stamped_start.pose   = start;
    start_node->path_segment.push_back(stamped_start);
    cost_so_far[start_idx] = 0.0;
    open.push(start_node);


    NodePtr goal_node = nullptr;
    double step = res;

    while(!open.empty()){
        auto cur = open.top(); open.pop();
        size_t cur_idx = cur->index;

        // Check if the node is already closed
        if (cur->g > cost_so_far[cur_idx]) continue;

        vector<geometry_msgs::msg::PoseStamped> shot_seg;
        if(tryShotToGoal(cur, goal, occupancy_grid, shot_seg)){
            // build final path
            vector<geometry_msgs::msg::PoseStamped> full;
            // prepend cur's ancestors
            for(auto p = cur; p; p = p->parent){
                full.insert(full.begin(), p->path_segment.begin(), p->path_segment.end());
            }
            // append shot segment
            full.insert(full.end(), shot_seg.begin(), shot_seg.end());
            path.poses = std::move(full);
            return path;
        }

        // Goal check
        double dx = cur->real_pose.position.x - goal.position.x;
        double dy = cur->real_pose.position.y - goal.position.y;
        double dist_to_goal = std::hypot(dx, dy);

        double goal_theta = tf2::getYaw(goal.orientation);
        double cur_theta  = tf2::getYaw(cur->real_pose.orientation);
        double angle_diff = std::fabs(normalizeAngle(goal_theta - cur_theta));

        // Torlerance
        double pos_tol = 0.5;                  
        double ang_tol = 5.0 * M_PI / 180.0;   

        if (dist_to_goal < pos_tol && angle_diff < ang_tol) {
            goal_node = cur;
            break;
        }

        // Close enough to the goal
        // double dist_to_goal = hypot(cur.x - goal_i, cur.y - goal_j);
        //if (grid_x == goal_i && grid_y == goal_j) {
        //   goal_node = cur;
        //  found_path = true;
        // break;
        // }


        // Check the 11 possible delta angles for the steering direction
        for (int delta : {-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5}) {

            geometry_msgs::msg::Pose start_pose;
            if (cur->parent == nullptr) {
                start_pose = cur->real_pose;
            } else {
                start_pose = cur->path_segment.back().pose;
            }
            double yaw = tf2::getYaw(start_pose.orientation);
            double theta = yaw + delta * angle_bin_size_;

            // build local goal_pose
            geometry_msgs::msg::Pose goal_pose;
            goal_pose.position.x = start_pose.position.x + step*cos(theta);
            goal_pose.position.y = start_pose.position.y + step*sin(theta);
            tf2::Quaternion q1; q1.setRPY(0,0,theta);
            goal_pose.orientation = tf2::toMsg(q1);

            // Attempt to move using the Reeds-Shepp model
            auto result = tryReedsSheppMove(
                start_pose, goal_pose, occupancy_grid, 
                res, allow_unknown_, obstacle_threshold_);

            if (!result.has_value()) continue;

            // Get the resulting pose
            double gx = result->end_pose.pose.position.x;
            double gy = result->end_pose.pose.position.y;

            // Convert back to grid
            int new_i = static_cast<int>(std::floor((gx - origin_x)/ res));
            int new_j = static_cast<int>(std::floor((gy - origin_y)/ res));
            if (new_i < 0 || new_i >= width || new_j < 0 || new_j >= height) continue;

            double goal_yaw = tf2::getYaw(result->end_pose.pose.orientation);
            int new_theta_idx = getAngleBinIndex((goal_yaw - origin_yaw), num_angle_bins_);
            size_t new_index = getIndex(new_i, new_j, new_theta_idx, width, num_angle_bins_);
            double new_g = cur->g + step;

            // Calculate the heuristic to the goal
            double dx_goal = (goal_i - new_i) * res;
            double dy_goal = (goal_j - new_j) * res;
            double new_h = std::hypot(dx_goal, dy_goal);

            // If the new path is better, update cost and push to open
            if (!cost_so_far.count(new_index) || new_g < cost_so_far[new_index]) {
                cost_so_far[new_index] = new_g;
                auto new_node = std::make_shared<Node3D>();

                new_node->real_pose = result->end_pose.pose;
                new_node->g = new_g;
                new_node->f = new_g + new_h;
                new_node->index = new_index; 
                new_node->parent = cur;
                new_node->path_segment = result->path_segment;

                // node_cache[new_index] = new_node;
                open.push(new_node);
            }
        }
    }


    std::vector<geometry_msgs::msg::PoseStamped> full_path;
    for (auto p = goal_node; p; p = p->parent) {
        const auto& seg = p->path_segment;
        if (seg.empty()) continue;
        if (full_path.empty()) {
            full_path.insert(full_path.begin(), seg.begin(), seg.end());
        } else {
            full_path.insert(full_path.begin(), seg.begin(), seg.end() - 1);
        }
    }

    if (!shot_seg.empty()) {
        full_path.insert(full_path.end(), shot_seg.begin() + 1, shot_seg.end());
    }

    path.poses = std::move(full_path);
    return path;
}  

}// namespace hybrid_a_star_planner
