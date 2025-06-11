#ifndef HYBRID_A_STAR_PLANNER__HYBRID_A_STAR_HPP_
#define HYBRID_A_STAR_PLANNER__HYBRID_A_STAR_HPP_

#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <limits>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace hybrid_a_star_planner
{

/**
 * @brief Hybrid A* planner using discrete SE(2) search with Reeds-Shepp expansion
 */
class HybridAStar
{
public:
    /**
     * @brief Construct a new HybridAStar object
     * @param allow_unknown Whether unknown cells (-1) are treated as free
     * @param obstacle_threshold OccupancyGrid threshold above which a cell is considered an obstacle
     */
    HybridAStar(bool allow_unknown = false, int obstacle_threshold = 50);

    ~HybridAStar() = default;

    /**
     * @brief Plan a path on a given occupancy grid map
     * @param occupancy_grid Input map (nav_msgs::msg::OccupancyGrid)
     * @param start Start pose in world frame
     * @param goal Goal pose in world frame
     * @return nav_msgs::msg::Path Smoothed path in world frame (x, y, z + quaternion)
     */
    nav_msgs::msg::Path makePlan(
        const nav_msgs::msg::OccupancyGrid &occupancy_grid,
        const geometry_msgs::msg::Pose &start,
        const geometry_msgs::msg::Pose &goal);

private:
    struct Node3D {
        
        geometry_msgs::msg::Pose real_pose; 
        double g;        // Cost-to-come (path cost from start)
        double f;        // Total cost (g + h)
        size_t index;  

        // Pointer to parent node for path reconstruction   
        std::shared_ptr<Node3D> parent;   

        // Store the path segment
        std::vector<geometry_msgs::msg::PoseStamped> path_segment;
    };

    struct CompareNodes {
    bool operator()(const std::shared_ptr<Node3D>& a,
                    const std::shared_ptr<Node3D>& b) const {
        return a->f > b->f;
    }
    };

    // Configuration parameters
    bool allow_unknown_;
    int obstacle_threshold_;
    int num_angle_bins_;
    double angle_bin_size_;
};

}  // namespace hybrid_a_star_planner

#endif  // HYBRID_A_STAR_PLANNER__HYBRID_A_STAR_HPP_
