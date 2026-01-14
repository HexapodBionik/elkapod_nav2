//
// Created by Damian Baraniak.
//
// Copyright (c) 2026.
// Elkapod Bionik, Warsaw University of Technology. All rights reserved.
//

#ifndef FOOTPRINT_TF_HPP
#define FOOTPRINT_TF_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/transform_broadcaster.h"

using Float64 = std_msgs::msg::Float64;

class FootprintTFPublisher : public rclcpp::Node{
    public:
    FootprintTFPublisher();

    private:
    double baseHeight;
    void baseHeightCmdCallback(const Float64& base_height);
    void timerCallback();
    rclcpp::Subscription<Float64>::SharedPtr base_height_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};
#endif