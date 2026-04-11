#include "../include/elkapod_slam/footprint_tf_publisher.hpp"
using namespace std::chrono_literals;

static const std::string base_height_param = "start_base_height";
static const std::string base_height_cmd_topic_param = "/cmd_base_height";

FootprintTFPublisher::FootprintTFPublisher() : Node("footprint_tf_publisher") {
  this->declare_parameter<double>(base_height_param, 0.120);
  base_height_sub_ = this->create_subscription<Float64>(
      base_height_cmd_topic_param, 10,
      std::bind(&FootprintTFPublisher::baseHeightCmdCallback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(50ms, std::bind(&FootprintTFPublisher::timerCallback, this));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  baseHeight = get_parameter(base_height_param).as_double();
}

void FootprintTFPublisher::baseHeightCmdCallback(const Float64& base_height) {
  baseHeight = base_height.data;
}

void FootprintTFPublisher::timerCallback() {
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "base_link";
  t.child_frame_id = "base_footprint";

  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = -this->baseHeight;

  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = 0.0;
  t.transform.rotation.w = 1.0;

  tf_broadcaster_->sendTransform(t);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FootprintTFPublisher>());
  rclcpp::shutdown();
  return 0;
}