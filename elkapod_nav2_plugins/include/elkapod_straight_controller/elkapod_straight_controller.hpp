#ifndef ELKAPOD_STRAIGHT_CONTROLLER_HPP_
#define ELKAPOD_STRAIGHT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace elkapod_straight_controller {
enum ControllerState { ROTATION, LINEAR };

using PoseStamped = geometry_msgs::msg::PoseStamped;
using Path = nav_msgs::msg::Path;
using Point = geometry_msgs::msg::Point;

class ElkapodStraightController : public nav2_core::Controller {
 public:
  ElkapodStraightController() = default;
  ~ElkapodStraightController() override = default;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                 const std::shared_ptr<tf2_ros::Buffer> tf,
                 const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& velocity,
      nav2_core::GoalChecker* goal_checker) override;

  void setPlan(const nav_msgs::msg::Path& path) override;

 protected:
  void shortenPath(nav_msgs::msg::Path& path, const PoseStamped& base_pose);
  void updateOrientationPath(nav_msgs::msg::Path& path);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("ElkapodStraightController")};
  rclcpp::Clock::SharedPtr clock_;

  double lookahead_dist_;
  double max_linear_vel;
  double max_angular_vel_;
  rclcpp::Duration transform_tolerance_{0, 0};
  ControllerState state_;
  nav_msgs::msg::Path global_plan_;
  nav_msgs::msg::Path shorten_plan_;


  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> simple_plan_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>> distance_left_pub_;

};

}  // namespace elkapod_straight_controller

#endif