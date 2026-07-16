#pragma once
#include <vector>
#include <cmath>
//#include <Eigen/Dense>
#include <fstream>
#include <limits.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "lart_msgs/msg/path_spline.hpp"
#include "lart_msgs/msg/cone.hpp"
#include "lart_msgs/msg/cone_array.hpp"
#include "lart_msgs/msg/dynamics.hpp"
#include "lart_common.h"
#include "topics.h"
#include "utils.hpp"
#include "types.hpp"


class skidpad_node : public rclcpp::Node
{
    public:
     skidpad_node();
    private:
        std::size_t last_idx_ = 0;
        std::vector<PathStruct> map;
        bool map_Localized = false;
        CarData carData;
        lart_msgs::msg::ConeArray::SharedPtr coneArray;
       
        //MAP LOCALIZER
        const double LOCK_THRESHOLD = 0.15;
        const size_t MAP_LOCALIZER_TRYS = 3000;
        
        double best_map_distance = std::numeric_limits<double>::max();
        size_t map_trys =0;

        double prev_corr_x_ = 0.0;
        double prev_corr_y_ = 0.0;

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_vis_pub;
        rclcpp::Publisher<lart_msgs::msg::PathSpline>::SharedPtr path_control_pub; 

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_subscriber;
        rclcpp::Subscription<lart_msgs::msg::ConeArray>::SharedPtr cone_array_subscriber;
        rclcpp::Subscription<lart_msgs::msg::Dynamics>::SharedPtr rpm_subscriber;
        


        void positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void coneArrayCallback(const lart_msgs::msg::ConeArray::SharedPtr msg);
        //enviar isto num struct é melhor roll pitch e yaw 
        void SplitLineSender();
        void track_correction(lart_msgs::msg::PathSpline *path,nav_msgs::msg::Path *path_vis);
        void RpmCallback(const lart_msgs::msg::Dynamics msg);


};