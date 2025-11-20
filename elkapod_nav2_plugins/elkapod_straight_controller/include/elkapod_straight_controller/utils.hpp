#include "geometry_msgs/msg/point.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav_msgs/msg/path.hpp"

using PoseStamped = geometry_msgs::msg::PoseStamped;
using Path = nav_msgs::msg::Path;
using Point = geometry_msgs::msg::Point;

inline Point getPosition(PoseStamped pose) { return pose.pose.position; }

double point_line_distance(const PoseStamped& point, const PoseStamped& start,
                           const PoseStamped& end) {
  double n = std::abs((getPosition(end).x - getPosition(start).x) *
                          (getPosition(start).y - getPosition(point).y) -
                      (getPosition(start).x - getPosition(point).x) *
                          ((getPosition(end).y - getPosition(start).y)));

  double d = nav2_util::geometry_utils::euclidean_distance(end, start);

  return n / d;
}

// Path rdp(const Path& points, double epsilon) {
//   double dmax = 0;
//   int32_t index = 0;
//   int32_t end = points.poses.size();
//   for (int32_t i = 2; i < end - 1; ++i) {
//     double d = point_line_distance(points.poses[i], points.poses[1], points.poses[end - 1]);
//     if (d > dmax) {
//       index = i;
//       dmax = d;
//     }
//   }

//   Path resultPath = Path();
//   resultPath.set__header(points.header);
//   if(dmax > epsilon){
//     // PathrecResults = rdp()
//   }
// }

double calculateSegmentAngle(const Point& p1, const Point& p2) {
  return std::atan2(p2.y - p1.y, p2.x - p1.x);
}
double calculateSegmentAngle(const PoseStamped& p1, const PoseStamped& p2) {
  return std::atan2(p2.pose.position.y - p1.pose.position.y,
                    p2.pose.position.x - p1.pose.position.x);
}
// std::vector<Point> simplifyPath(const Path& path) {
//   const double eps = 0.0001;
//   std::vector<Point> pointsExtracted;
//   std::transform(path.poses.begin(), path.poses.end(), pointsExtracted.begin(),
//                  [](const PoseStamped& x) { return x.pose.position; });

//   std::vector<Point> result = {pointsExtracted[0]};
//   double currHeading = calculateSegmentAngle(pointsExtracted[0], pointsExtracted[1]);

//   for (auto it = pointsExtracted.begin() + 1, it2 = it + 1; it2 != pointsExtracted.end();
//        ++it, ++it2) {
//     double newHeading = calculateSegmentAngle(*it, *it2);
//     if (std::fabs(currHeading - newHeading) > eps) {
//       currHeading = newHeading;
//       result.push_back(*it2);
//     }
//   }
//   return result;
// }

Path simplifyPath(const Path& path) {
  const double eps = 0.0001;
  Path simplePath = Path();
  simplePath.set__header(path.header);
  simplePath.poses = {path.poses[0]};
  double currHeading = calculateSegmentAngle(path.poses[0], path.poses[1]);

  for (auto it = path.poses.begin() + 1, it2 = it + 1; it2 != path.poses.end(); ++it, ++it2) {
    double newHeading = calculateSegmentAngle(*it, *it2);
    if (std::fabs(currHeading - newHeading) > eps) {
      currHeading = newHeading;
      simplePath.poses.push_back(*it2);
    }
  }

  return simplePath;
}

void trimPath(Path& plan, PoseStamped base_pose, double distTreshold) {
  // Assumes plan and base_pose are in the same frame
  auto last_point = std::find_if(plan.poses.begin(), plan.poses.end(), [&](PoseStamped pose) {
    return nav2_util::geometry_utils::euclidean_distance(pose, base_pose, false) > distTreshold;
  });

  plan.poses.erase(last_point, plan.poses.end());
}

double shortest_angle_diff(double goal, double current) {
  double d = std::remainder(goal - current, 2.0 * M_PI);
  return d;
}

double easeOutCubic(double x) { return 1 - pow(1 - x, 3); }