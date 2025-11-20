#include "elkapod_straight_controller/elkapod_straight_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>

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

/**
 * Find element in iterator with the minimum calculated value
 */
template <typename Iter, typename Getter>
Iter min_by(Iter begin, Iter end, Getter getCompareVal) {
  if (begin == end) {
    return end;
  }
  auto lowest = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it) {
    auto comp = getCompareVal(*it);
    if (comp < lowest) {
      lowest = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}

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
  (void)velocity;
  (void)goal_checker;
  geometry_msgs::msg::TwistStamped cmd_vel;
  double rotationDiff =
      shortest_angle_diff(calculateSegmentAngle(pose, simple_plan_.poses[1]), tf2::getYaw(pose.pose.orientation));

  if (std::abs(rotationDiff) <= eps) {
    double distance = nav2_util::geometry_utils::euclidean_distance(pose, simple_plan_.poses[1]);
    double speed = distance >= 1 ? max_linear_vel : max_linear_vel * easeOutCubic(distance);
    cmd_vel.twist.linear.set__x(speed);
  } else {
    double speed =
        rotationDiff > 0.5 ? max_angular_vel_ : max_angular_vel_ * easeOutCubic(2 * rotationDiff);
    cmd_vel.twist.angular.set__z(speed);
  }
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();
  return cmd_vel;
}

void ElkapodStraightController::setPlan(const nav_msgs::msg::Path& path) {
  global_plan_ = path;
  PoseStamped robot_pose;
  nav2_util::getCurrentPose(robot_pose, *tf_);
  trimPath(global_plan_, robot_pose, lookahead_dist_);
  simple_plan_ = simplifyPath(global_plan_);
  RCLCPP_INFO(logger_, "Original plan size %ld, trimmed plan size %ld, simplified plan size %ld",
              path.poses.size(), global_plan_.poses.size(), simple_plan_.poses.size());
  std::cout << global_plan_.poses.size() << "  " << path.poses.size() << std::endl;
  global_pub_->publish(global_plan_);
  simple_plan_pub_->publish(simple_plan_);
}

nav_msgs::msg::Path ElkapodStraightController::transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped& pose) {
  // Original mplementation taken fron nav2_dwb_controller

  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(tf_, global_plan_.header.frame_id, pose, robot_pose, transform_tolerance_)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // We'll discard points on the plan that are outside the local costmap
  nav2_costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
  double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
                          costmap->getResolution() / 2.0;

  // First find the closest pose on the path to the robot
  auto transformation_begin = min_by(global_plan_.poses.begin(), global_plan_.poses.end(),
                                     [&robot_pose](const geometry_msgs::msg::PoseStamped& ps) {
                                       return euclidean_distance(robot_pose, ps);
                                     });

  // From the closest point, look for the first point that's further then dist_threshold from the
  // robot. These points are definitely outside of the costmap so we won't transform them.
  auto transformation_end = std::find_if(
      transformation_begin, end(global_plan_.poses), [&](const auto& global_plan_pose) {
        return euclidean_distance(robot_pose, global_plan_pose) > dist_threshold;
      });

  // Helper function for the transform below. Transforms a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto& global_plan_pose) {
    // We took a copy of the pose, let's lookup the transform at the current time
    geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
    stamped_pose.header.frame_id = global_plan_.header.frame_id;
    stamped_pose.header.stamp = pose.header.stamp;
    stamped_pose.pose = global_plan_pose.pose;
    transformPose(tf_, costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose,
                  transform_tolerance_);
    return transformed_pose;
  };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(transformation_begin, transformation_end,
                 std::back_inserter(transformed_plan.poses), transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  global_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

bool ElkapodStraightController::transformPose(const std::shared_ptr<tf2_ros::Buffer> tf,
                                              const std::string frame,
                                              const geometry_msgs::msg::PoseStamped& in_pose,
                                              geometry_msgs::msg::PoseStamped& out_pose,
                                              const rclcpp::Duration& transform_tolerance) const {
  // Implementation taken as is fron nav_2d_utils in nav2_dwb_controller

  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf->transform(in_pose, out_pose, frame);
    return true;
  } catch (tf2::ExtrapolationException& ex) {
    auto transform = tf->lookupTransform(frame, in_pose.header.frame_id, tf2::TimePointZero);
    if ((rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) >
        transform_tolerance) {
      RCLCPP_ERROR(rclcpp::get_logger("tf_help"),
                   "Transform data too old when converting from %s to %s",
                   in_pose.header.frame_id.c_str(), frame.c_str());
      RCLCPP_ERROR(rclcpp::get_logger("tf_help"), "Data time: %ds %uns, Transform time: %ds %uns",
                   in_pose.header.stamp.sec, in_pose.header.stamp.nanosec,
                   transform.header.stamp.sec, transform.header.stamp.nanosec);
      return false;
    } else {
      tf2::doTransform(in_pose, out_pose, transform);
      return true;
    }
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(rclcpp::get_logger("tf_help"), "Exception in transformPose: %s", ex.what());
    return false;
  }
  return false;
}

}  // namespace elkapod_straight_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(elkapod_straight_controller::ElkapodStraightController,
                       nav2_core::Controller)