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
    double total_dist = 0;
};

//Responsible for sending planned path marker and planned path topic
void skidpad_node::SplitLineSender(CarData carData){
    double dist = 0;
    auto stamp = this->now();
    
    //messages inicialization
    geometry_msgs::msg::PoseStamped pose;
    lart_msgs::msg::PathSpline pathSpline_msg;

    pathSpline_msg.header.stamp = stamp;
    pathSpline_msg.header.frame_id = "world";

    nav_msgs::msg::Path path_rviz_msg;
    path_rviz_msg.header.stamp = stamp;
    path_rviz_msg.header.frame_id = "world";

    for(std::size_t i = 0; i<map.size();i++){
        double dx = map[i].x - carData.car_x;
        double dy = map[i].y - carData.car_y;

        double forward =
            dx * std::cos(carData.yaw) +
            dy * std::sin(carData.yaw);

        if(forward < 0.0)
            continue;


        // if(map[i].x < carData.car_x && map[i].y < carData.car_y)//Corrigir 
        //     continue;
        
        if(dist > 100){
            //RCLCPP_INFO(this->get_logger(), "Enviar");
            path_control_pub->publish(pathSpline_msg);
            path_vis_pub->publish(path_rviz_msg);
            break;
        }

        for(std::size_t j = 1; j<map.size();j++){
            double d = distance(map[i].x, map[i].y, map[j].x,map[j].y);

            if(d <= 0.5){
                geometry_msgs::msg::PoseStamped pose;
                pose = createPoseMsg(map[j].x,map[j].y,
                    carData.roll, carData.pitch,carData.yaw,
                    stamp); 

                //MEGA DEBUG MESSAGE
                // RCLCPP_INFO(this->get_logger(), 
                // "Novo Ponto Adicionado:\n"
                // "  Car Pos x:%.2f   y:%.2f"
                // "  Distância Total: %.2f\n"
                // "  Ponto a atual a envia x:%.2f y:%.2f"
                // "  Curvatura: %.4f\n",
                // carData.car_x,
                // carData.car_y,
                // dist,
                // map[j].x,
                // map[j].y,
                // map[j].cur);

                pathSpline_msg.poses.push_back(pose);
                pathSpline_msg.curvature.push_back(map[j].cur);
                path_rviz_msg.poses.push_back(pose);
                dist += d;
                pathSpline_msg.distance.push_back(dist);
            }
        }
    }
}



//Sem localizar o mapa primeiro
void skidpad_node::positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    CarData carData;
    
    
    carData.car_x = msg->pose.position.x;
    carData.car_y = msg->pose.position.y;

    tf2::Quaternion q(
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z,
    msg->pose.orientation.w
    );

    tf2::Matrix3x3 m(q);
    m.getRPY(carData.roll, carData.pitch, carData.yaw);
    
    if(map_Localized){
        SplitLineSender(carData);
    }else{
        RCLCPP_INFO(this->get_logger(), "Map is not localized");
    }
}


void skidpad_node::coneArrayCallback(const lart_msgs::msg::ConeArray::SharedPtr msg)
{
    auto cones_s = msg->cones;
    if(!map_Localized){
        RCLCPP_INFO(this->get_logger(), "Trying to localize the car");

        double dist_b = std::numeric_limits<double>::max();
        double dist_y = std::numeric_limits<double>::max();
        int blue_index = -1;
        int yellow_index = -1;

        double tmp_distance_blue = 10, tmp_distance_yellow = 10;
        if(!cones_s.empty()){
            for (size_t i = 0; i < cones_s.size(); i++){
                if (cones_s[i].BLUE == 2){
                    tmp_distance_blue = distance(cones_s[i].position.x, cones_s[i].position.y, 0, 0);//Verificar se o carro começa em 0
                    if (tmp_distance_blue < dist_b){
                        dist_b = tmp_distance_blue;
                        blue_index = i;
                    }
                }else if (cones_s[i].YELLOW == 1){
                    tmp_distance_yellow = distance(cones_s[i].position.x, cones_s[i].position.y, 0, 0);
                    if (tmp_distance_yellow < dist_y){
                        dist_y = tmp_distance_yellow;
                        yellow_index = i;
                    }
                }
            }
            //Calcula o ponto medio dos cones mais proximos ao caroo
            if (blue_index != -1 && yellow_index != -1) 
            {
                map_localizer(msg,blue_index,yellow_index,&map);
                map_Localized = true;
            }
        }
    }
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<skidpad_node>());
    rclcpp::shutdown();
    return 0;
}