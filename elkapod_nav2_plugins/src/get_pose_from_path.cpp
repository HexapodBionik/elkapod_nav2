#include "get_pose_from_path/get_pose_from_path.hpp"

#include <string>
#include <memory>
#include <cmath>

#include "behaviortree_cpp/action_node.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behavior_tree {

GetPoseFromPath::GetPoseFromPath(
    const std::string & name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf)
  {}


  BT::NodeStatus GetPoseFromPath::tick()
  {
    nav_msgs::msg::Path path;
    if (!getInput("input_path", path)) {
      return BT::NodeStatus::FAILURE;
    }

    if (path.poses.empty()) {
      return BT::NodeStatus::FAILURE;
    }

    double lookahead_dist;
    if (!getInput("lookahead_dist", lookahead_dist)) {
      lookahead_dist = 1.0;
    }

    // --- Logic: Find point X meters away ---
    geometry_msgs::msg::PoseStamped target_pose = path.poses.back(); // Default to end
    double accumulated_dist = 0.0;

    // Iterate starting from the second point
    for (size_t i = 0; i < path.poses.size() - 1; ++i) {
      const auto & p1 = path.poses[i].pose.position;
      const auto & p2 = path.poses[i + 1].pose.position;

      double dx = p1.x - p2.x;
      double dy = p1.y - p2.y;
      double segment_dist = std::sqrt(dx * dx + dy * dy);

      accumulated_dist += segment_dist;

      if (accumulated_dist >= lookahead_dist) {
        target_pose = path.poses[i + 1];
        break;
      }
    }

    setOutput("output_pose", target_pose);
    return BT::NodeStatus::SUCCESS;
  }


}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GetPoseFromPath>(
    "GetPoseFromPath");
}