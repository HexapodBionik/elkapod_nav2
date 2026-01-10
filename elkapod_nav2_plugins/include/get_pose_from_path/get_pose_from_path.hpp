#ifndef GET_POSE_FROM_PATH_BT_NODE
#define GET_POSE_FROM_PATH_BT_NODE


#include <string>
#include <memory>
#include <cmath>

#include "behaviortree_cpp/action_node.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behavior_tree
{

class GetPoseFromPath : public BT::SyncActionNode
{
public:
  GetPoseFromPath(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path", "The global path to process"),
      BT::InputPort<double>("lookahead_dist", 1.0, "Distance to look ahead along the path"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_pose", "The extracted target pose")
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace nav2_behavior_tree

#endif