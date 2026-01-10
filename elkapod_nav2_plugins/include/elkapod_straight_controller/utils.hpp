#include "geometry_msgs/msg/point.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav_msgs/msg/path.hpp"

using PoseStamped = geometry_msgs::msg::PoseStamped;
using Path = nav_msgs::msg::Path;
using Point = geometry_msgs::msg::Point;


inline double calculateSegmentAngle(const Point& p1, const Point& p2) {
  return std::atan2(p2.y - p1.y, p2.x - p1.x);
}

inline double calculateSegmentAngle(const PoseStamped& p1, const PoseStamped& p2) {
  return std::atan2(p2.pose.position.y - p1.pose.position.y,
                    p2.pose.position.x - p1.pose.position.x);
}

inline double easeOutCubic(double x) { return 1 - pow(1 - x, 3); }

inline double calculateRotationDiff(const PoseStamped& p1, const PoseStamped& p2) {
  double yaw1 = tf2::getYaw(p1.pose.orientation);
  double yaw2 = tf2::getYaw(p2.pose.orientation);
  return yaw1 - yaw2;
}