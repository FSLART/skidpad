#pragma once
#include "my_node.hpp"
#include "types.hpp"

double distance(double x, double y, double x1, double y1);

geometry_msgs::msg::PoseStamped createPoseMsg(
    double x, double y, 
    double roll, double pitch, double yaw, 
    const rclcpp::Time& stamp);

void map_localizer(const lart_msgs::msg::ConeArray::SharedPtr msg, int blue_index,int yellow_index,int gate_index1, int gate_index2, std::vector<PathStruct> *map);
std::vector<PathStruct> file_loader(std::string fileName);