#pragma once

#include <vector>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_abc_pso_planner
{
class ABCPSOPlanner : public nav2_core::GlobalPlanner
{
public:
  ABCPSOPlanner();
  ~ABCPSOPlanner();

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  struct BeeParticle {
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> best_position;
    double cost;
  };

  std::vector<BeeParticle> initializeSwarm(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

  double evaluateCost(const std::vector<double> & path);
  nav_msgs::msg::Path convertToNavPath(const std::vector<double> & path);

  rclcpp::Logger logger_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  int swarm_size_;
  int max_iterations_;
  double robot_radius_;
  geometry_msgs::msg::PoseStamped current_start_;
  geometry_msgs::msg::PoseStamped current_goal_;

  // PSO-specific parameters
  double w_;       // Inertia weight
  double wdamp_;   // Inertia weight damping
  double c1_;      // Cognitive coefficient
  double c2_;      // Social coefficient

  // Cost-related weights
  double w_length_;    // Weight for path length
  double w_offmap_;    // Weight for off-map penalty
  double w_cell_;      // Weight for cell cost
  double w_unknown_;   // Weight for unknown cells
  double w_lethal_;    // Weight for lethal obstacles
  double w_inflated_;  // Weight for inflated obstacles
  int num_waypoints_;  // Number of waypoints in the path

  // Cached plan information
  double cached_best_cost_;
  nav_msgs::msg::Path cached_path_;
  geometry_msgs::msg::PoseStamped prev_goal_;
};
}  // namespace nav2_abc_pso_planner

