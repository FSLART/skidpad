#pragma once
#include "my_node.hpp"
#include "types.hpp"

inline double distance(double x, double y, double x1, double y1);
inline geometry_msgs::msg::PoseStamped createPoseMsg(
    double x, double y, 
    double roll, double pitch, double yaw, 
    const rclcpp::Time& stamp, 
    const std::string& frame_id = "world");

std::vector<PathStruct> file_loader(std::string fileName);