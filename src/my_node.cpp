#include "../include/my_node.hpp"

using std::placeholders::_1;

skidpad_node::skidpad_node() : Node("skidpadNode")
{
    RCLCPP_INFO(this->get_logger(), "Skidpad node has been started");

    this->path_vis_pub = this->create_publisher<nav_msgs::msg::Path>("/planned_path_marker", 10);
    this->path_control_pub = this->create_publisher<lart_msgs::msg::PathSpline>("/planned_path_topic", 10);

    this->cone_array_subscriber = this->create_subscription<lart_msgs::msg::ConeArray>("/mapping/cones", 10, std::bind(&skidpad_node::coneArrayCallback, this, _1));
    this->position_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("/slam/pose", 10, std::bind(&skidpad_node::positionCallback, this, _1));

    map = file_loader("skidpad_path.csv");
};

//Sem localizar o mapa primeiro
void skidpad_node::positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    double car_pos_x = msg->pose.position.x;
    double car_pos_y = msg->pose.position.y;
    double dist = 0;


    lart_msgs::msg::PathSpline pathSpline_msg;
    pathSpline_msg.header.stamp = this->now();
    pathSpline_msg.header.frame_id = "world";

    nav_msgs::msg::Path path_rviz_msg;
    path_rviz_msg.header.stamp = this->now();
    path_rviz_msg.header.frame_id = "world";

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = "world";

    for(PathStruct path : map){
        if(path.x < car_pos_x && path.y < car_pos_y)//propenso a erros bom para testar mau para o resto
            continue;

        pose.header.frame_id = "world";
        pose.pose.position.x = path.x;
        pose.pose.position.y = path.y;
        pose.pose.position.z = 0;

        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, 0.3);

        pose.pose.orientation.x = quaternion.x();
        pose.pose.orientation.y = quaternion.y();
        pose.pose.orientation.z = quaternion.z();
        pose.pose.orientation.w = quaternion.w();

        

        pathSpline_msg.poses.push_back(pose);
        pathSpline_msg.distance.push_back(dist);
        pathSpline_msg.curvature.push_back(path.cur);

        
        path_rviz_msg.poses.push_back(pose);




        dist = dist+0.5;


        //atenção a distancia 
        if(dist > 40){
            RCLCPP_INFO(this->get_logger(), "Enviar");

            path_control_pub->publish(pathSpline_msg);
            path_vis_pub->publish(path_rviz_msg);
            break;
        }
    }

}


void skidpad_node::coneArrayCallback(const lart_msgs::msg::ConeArray::SharedPtr msg){

}



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<skidpad_node>());
    rclcpp::shutdown();
    return 0;
}