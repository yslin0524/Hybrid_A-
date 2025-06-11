#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "hybrid_a_star_planner/hybrid_a_star.hpp"
#include "hybrid_a_star_planner/planner_node.hpp"
#include "hybrid_a_star_planner/utils/planner_utils.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using namespace std::chrono_literals;

namespace hybrid_a_star_planner {

HybridAStarPlannerNode::HybridAStarPlannerNode()
    : rclcpp::Node("hybrid_a_star_planner_node"), has_map_(false),
      has_initial_pose_(false), has_goal_(false), planner_() {
  // Declare and get parameters
  bool allow_unknown = this->declare_parameter<bool>("allow_unknown", true);
  int obstacle_threshold =
      this->declare_parameter<int>("obstacle_threshold", 50);
  planner_ = HybridAStar(allow_unknown, obstacle_threshold);

  // Publishers and subscribers
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/planned_path", rclcpp::QoS(1).reliable());

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&HybridAStarPlannerNode::onMapReceived, this,
                std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", rclcpp::QoS(1).reliable(),
      std::bind(&HybridAStarPlannerNode::onGoalReceived, this,
                std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/gt_pose",
      rclcpp::QoS(10),
      std::bind(&HybridAStarPlannerNode::onInitialPoseReceived, this, std::placeholders::_1));
  
  

  /*initial_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/wamv/sensors/position/ground_truth_odometry",
    rclcpp::QoS(10),
    std::bind(&HybridAStarPlannerNode::onInitialPoseReceived, this, std::placeholders::_1));
    */

  RCLCPP_INFO(this->get_logger(), "HybridAStarPlannerNode initialized.");

  
}

HybridAStarPlannerNode::~HybridAStarPlannerNode() {}

// 
void HybridAStarPlannerNode::onMapReceived(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_map_ = *msg;
  has_map_ = true;

  RCLCPP_INFO(this->get_logger(), "Received map [%d x %d], resolution: %.3f",
              latest_map_.info.width, latest_map_.info.height,
              latest_map_.info.resolution);

  updatePlan();
}

void HybridAStarPlannerNode::onInitialPoseReceived(
    // const nav_msgs::msg::Odometry::SharedPtr msg)
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  initial_pose_.header = msg->header;
  initial_pose_.pose = msg->pose;

  // initial_pose_.pose = msg->pose.pose;
  has_initial_pose_ = true;

  RCLCPP_DEBUG(this->get_logger(),
               "Initial pose (from odometry) set to (%.2f, %.2f).",
               initial_pose_.pose.position.x, initial_pose_.pose.position.y);

  // updatePlan();
}

void HybridAStarPlannerNode::onGoalReceived(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  latest_goal_ = *msg;
  has_goal_ = true;
  RCLCPP_INFO(this->get_logger(), "New goal received (%.2f, %.2f)",
              msg->pose.position.x, msg->pose.position.y);

  updatePlan();
}

void HybridAStarPlannerNode::updatePlan() {
  if (!has_map_ || !has_initial_pose_ || !has_goal_) {
    RCLCPP_WARN(
        this->get_logger(),
        "Missing required inputs (map/initial_pose/goal). Skip planning.");
    return;
  }

  geometry_msgs::msg::Pose start_pose = initial_pose_.pose;
  geometry_msgs::msg::Pose goal_pose = latest_goal_.pose;

  RCLCPP_INFO(this->get_logger(), "Planning path to goal (%.2f, %.2f)",
              goal_pose.position.x, goal_pose.position.y);

  nav_msgs::msg::Path path =
      planner_.makePlan(latest_map_, start_pose, goal_pose);

  path.header.frame_id = "map";
  path.header.stamp = this->now();

  if (path.poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "No path found.");
  } else {
    RCLCPP_INFO(this->get_logger(), "Path planned with %zu poses.",
                path.poses.size());
  }

  path_pub_->publish(path);
}

} // namespace hybrid_a_star_planner

// Main function
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hybrid_a_star_planner::HybridAStarPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
