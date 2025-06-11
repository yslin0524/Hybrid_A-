#ifndef HYBRID_A_STAR_PLANNER__PLANNER_NODE_HPP_
#define HYBRID_A_STAR_PLANNER__PLANNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>  // Odom
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include "hybrid_a_star_planner/hybrid_a_star.hpp"


namespace hybrid_a_star_planner
{

class HybridAStarPlannerNode : public rclcpp::Node
{
public:
  HybridAStarPlannerNode();
  ~HybridAStarPlannerNode();

private:
  void onMapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onGoalReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onInitialPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // void onInitialPoseReceived(const nav_msgs::msg::Odometry::SharedPtr msg);
  
  void updatePlan();  
  

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr initial_pose_sub_;

  // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr initial_pose_sub_;
  
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  nav_msgs::msg::OccupancyGrid latest_map_;
  bool has_map_;
  
  geometry_msgs::msg::PoseStamped initial_pose_;
  bool has_initial_pose_;
  
  geometry_msgs::msg::PoseStamped latest_goal_;
  bool has_goal_;


  HybridAStar planner_;
};

}  // namespace hybrid_a_star_planner

#endif  // HYBRID_A_STAR_PLANNER__PLANNER_NODE_HPP_
