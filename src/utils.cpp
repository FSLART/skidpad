#include "../include/utils.hpp"

inline double distance(double x, double y, double x1, double y1){
    return (double)std::sqrt((x1 - x) * (x1 - x) + (y1 - y) * (y1 - y));
}


inline geometry_msgs::msg::PoseStamped createPoseMsg(
    double x, double y, 
    double roll, double pitch, double yaw,
    const rclcpp::Time& stamp, 
    const std::string& frame_id = "world")
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = frame_id;

        pose.header.frame_id = "world";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0;

        tf2::Quaternion quaternion;
        quaternion.setRPY(roll, pitch, yaw);

        pose.pose.orientation.x = quaternion.x();
        pose.pose.orientation.y = quaternion.y();
        pose.pose.orientation.z = quaternion.z();
        pose.pose.orientation.w = quaternion.w();

        return pose;
    }



std::vector<PathStruct> file_loader(std::string fileName){
    std::ifstream File(fileName);
    if(!File.is_open())
    {
        std::cout << "Cannot open this file:" << fileName << std::endl;
        std::cout << "Trying more 5 times" << fileName << std::endl;
        for(int i = 0; i<5; i++){
            std::ifstream File(fileName);
            if(File.is_open()){
                break;
            }
        }    
    }
    
    std::vector<PathStruct>skidpad_map;
    
    std::string line;
    while (std::getline(File,line))
    {
        if (line.empty())
            continue;

        std::stringstream ss(line);
        std::string x_str, y_str, cur_str;

        if(std::getline(ss,x_str,',') && std::getline(ss,y_str,',') && std::getline(ss,cur_str)){
            PathStruct tmp;
            tmp.x = std::stod(x_str);
            tmp.y = std::stod(y_str);
            tmp.cur = std::stod(cur_str);
            skidpad_map.push_back(tmp);        
        }
    }
    return skidpad_map;
}