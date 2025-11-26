#include "elkapod_straight_controller/elkapod_straight_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "elkapod_straight_controller/utils.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using std::abs;
using std::hypot;
using std::max;
using std::min;

namespace elkapod_straight_controller {

void ElkapodStraightController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent;

  auto node = node_.lock();

  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  declare_parameter_if_not_declared(node, plugin_name_ + ".max_linear_vel",
                                    rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_dist",
                                    rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_angular_vel",
                                    rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".transform_tolerance",
                                    rclcpp::ParameterValue(0.1));

  node->get_parameter(plugin_name_ + ".max_linear_vel", max_linear_vel);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  double transform_tolerance;
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
  state_ = ROTATION;
  global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  simple_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("simplified_plan", 1);
}

void ElkapodStraightController::cleanup() {
  RCLCPP_INFO(
      logger_,
      "Cleaning up controller: %s of type pure_pursuit_controller::ElkapodStraightController",
      plugin_name_.c_str());
  global_pub_.reset();
  simple_plan_pub_.reset();
}

void ElkapodStraightController::activate() {
  RCLCPP_INFO(
      logger_,
      "Activating controller: %s of type pure_pursuit_controller::ElkapodStraightController\"  %s",
      plugin_name_.c_str(), plugin_name_.c_str());
  global_pub_->on_activate();
  simple_plan_pub_->on_activate();
}

void ElkapodStraightController::deactivate() {
  RCLCPP_INFO(
      logger_,
      "Dectivating controller: %s of type pure_pursuit_controller::ElkapodStraightController\"  %s",
      plugin_name_.c_str(), plugin_name_.c_str());
  global_pub_->on_deactivate();
  simple_plan_pub_->on_deactivate();
}

void ElkapodStraightController::setSpeedLimit(const double& speed_limit, const bool& percentage) {
  (void)speed_limit;
  (void)percentage;
  RCLCPP_INFO(logger_, "setSpeedLimit method has been invoked in %s", plugin_name_.c_str());
}

geometry_msgs::msg::TwistStamped ElkapodStraightController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& velocity,
    nav2_core::GoalChecker* goal_checker) {
  const double eps = 0.05;
  const size_t n = 10;
  (void)velocity;
  (void)goal_checker;
  geometry_msgs::msg::TwistStamped cmd_vel;
  auto closePointIter =
      (std::min_element(global_plan_.poses.begin(), global_plan_.poses.end(),
                        [&](const PoseStamped& a, const PoseStamped& b) {
                          return nav2_util::geometry_utils::euclidean_distance(a, pose) <
                                 nav2_util::geometry_utils::euclidean_distance(b, pose);
                        }));
  const PoseStamped closestPoint = *closePointIter;
  // double closestPointYaw = tf2::getYaw(closestPoint.pose.orientation);
  // double robotYaw = tf2::getYaw(pose.pose.orientation);
  // double rotationDiff = closestPointYaw - robotYaw;
  double rotationDiff = calculateRotationDiff(closestPoint, pose);
  if (abs(rotationDiff) < eps) {
    double frac = 1;
    auto remaining = static_cast<size_t>(global_plan_.poses.end() - closePointIter);
    auto end_it = std::next(closePointIter, std::min(n, remaining));
    auto found_it = std::find_if(closePointIter, end_it, [&](const auto& x) {
      return abs(calculateRotationDiff(x, pose)) >= eps;
    });
    if (found_it != end_it) {
      int dist = static_cast<int>(found_it - closePointIter);
      frac = dist / 10.0;
      frac = std::max(frac, 0.2);
        }
    cmd_vel.twist.linear.set__x(frac * max_linear_vel);
  } else {
    double angularVelocity = copysign(1.0, rotationDiff) * max_angular_vel_;
    double frac = (abs(rotationDiff) > 1.0) ? 1.0 : easeOutCubic(abs(rotationDiff));
    cmd_vel.twist.angular.set__z(frac * angularVelocity);
  }
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();
  return cmd_vel;
}

void ElkapodStraightController::setPlan(const nav_msgs::msg::Path& path) {
  global_plan_ = path;
  PoseStamped robot_pose;
  nav2_util::getCurrentPose(robot_pose, *tf_);
  shortenPath(global_plan_, robot_pose);
  updateOrientationPath(global_plan_);

  RCLCPP_INFO(logger_, "Original plan size %ld, trimmed plan size %ld", path.poses.size(),
              global_plan_.poses.size());
  global_pub_->publish(global_plan_);
}

void ElkapodStraightController::shortenPath(nav_msgs::msg::Path& path,
                                            const PoseStamped& base_pose) {
  auto last_point =
      std::find_if(path.poses.begin(), path.poses.end(), [&](const PoseStamped& pose) {
        return nav2_util::geometry_utils::euclidean_distance(pose, base_pose, false) >
               lookahead_dist_;
      });

  path.poses.erase(last_point, path.poses.end());
}
void ElkapodStraightController::updateOrientationPath(nav_msgs::msg::Path& path) {
  for (auto it1 = path.poses.begin(), it2 = it1 + 1; it2 != path.poses.end(); ++it1, ++it2) {
    double yawRequired = calculateSegmentAngle(*it1, *it2);
    const auto q = nav2_util::geometry_utils::orientationAroundZAxis(yawRequired);
    it1->pose.set__orientation(q);
  }
}
}  // namespace elkapod_straight_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(elkapod_straight_controller::ElkapodStraightController,
                       nav2_core::Controller)