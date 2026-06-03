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

void skidpad_node::SplitLineSender(){//CarData carData){
    auto stamp = this->now();
    
    // 1. Inicialização das mensagens
    lart_msgs::msg::PathSpline pathSpline_msg;
    pathSpline_msg.header.stamp = stamp;
    pathSpline_msg.header.frame_id = "world";

    nav_msgs::msg::Path path_rviz_msg;
    path_rviz_msg.header.stamp = stamp;
    path_rviz_msg.header.frame_id = "world";

    if (map.empty()) return; // Prevenção de segurança

    // 2. Encontrar o ponto de partida (o mais próximo que esteja à frente do carro)
    int start_idx = -1;
    for(std::size_t i = 0; i < map.size(); i++){
        double dx = map[i].x - carData.car_x;
        double dy = map[i].y - carData.car_y;
        
        // Produto escalar simples para saber se o ponto está à frente
        double forward = dx * std::cos(carData.yaw) + dy * std::sin(carData.yaw);

        if(forward > 0.0){
            start_idx = i;
            break; 
        }
    }

    if(start_idx == -1) {
        RCLCPP_WARN(this->get_logger(), "Nenhum ponto do mapa encontrado à frente do carro!");
        return; 
    }

    // Variáveis para controlar a distância entre pontos
    double last_added_x = map[start_idx].x;
    double last_added_y = map[start_idx].y;
    double cumulative_dist = 0.0;

    // Adicionar o primeiro ponto à mensagem
    geometry_msgs::msg::PoseStamped pose = createPoseMsg(
        map[start_idx].x, map[start_idx].y,
        carData.roll, carData.pitch, carData.yaw, stamp
    );
    pathSpline_msg.poses.push_back(pose);
    pathSpline_msg.curvature.push_back(map[start_idx].cur);
    pathSpline_msg.distance.push_back(cumulative_dist);
    path_rviz_msg.poses.push_back(pose);

    // 3. Iterar sequencialmente (usando while e módulo para lidar com o circuito fechado)
    std::size_t i = (start_idx + 1) % map.size(); // Começa no ponto a seguir
    std::size_t pontos_verificados = 0;           // Segurança contra loops infinitos

    while(pathSpline_msg.poses.size() < 40 && pontos_verificados < map.size()){
        
        // Distância ao ÚLTIMO ponto que enviámos
        double d = distance(last_added_x, last_added_y, map[i].x, map[i].y);

        // Só guarda se a distância for maior ou igual a 50cm
        if(d >= 0.5){
            pose = createPoseMsg(
                map[i].x, map[i].y,
                carData.roll, carData.pitch, carData.yaw, stamp
            );
            pose = skidpad_node::track_correction(pose);
            pathSpline_msg.poses.push_back(pose);
            pathSpline_msg.curvature.push_back(map[i].cur);
            
            cumulative_dist += d;
            pathSpline_msg.distance.push_back(cumulative_dist);
            path_rviz_msg.poses.push_back(pose);

            // Atualiza a âncora para o próximo cálculo de distância
            last_added_x = map[i].x;
            last_added_y = map[i].y;
        }

        // Avança para o próximo ponto (se chegar ao fim do map, volta a 0)
        i = (i + 1) % map.size();
        pontos_verificados++;
    }

    // 4. Publicar apenas UMA vez no final da função
    if(!pathSpline_msg.poses.empty()){
        // RCLCPP_INFO(this->get_logger(), "Enviado Path: %zu pontos, Distância total: %.2fm", 
        //             pathSpline_msg.poses.size(), cumulative_dist);
        path_control_pub->publish(pathSpline_msg);
        path_vis_pub->publish(path_rviz_msg);
    }
}


//Sem localizar o mapa primeiro
void skidpad_node::positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
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
        SplitLineSender();
    }else{
        RCLCPP_INFO(this->get_logger(), "Map is not localized");
    }
}


void skidpad_node::coneArrayCallback(const lart_msgs::msg::ConeArray::SharedPtr msg)
{
    auto cones_s = msg->cones;
    skidpad_node::coneArray = msg;

    if(!map_Localized){
        RCLCPP_INFO(this->get_logger(), "Trying to localize the car");

        double dist_b = std::numeric_limits<double>::max();
        double dist_y = std::numeric_limits<double>::max();
        double dist_o = std::numeric_limits<double>::max();

        int blue_index = -1;
        int yellow_index = -1;

        //Index of the first 2 orange cones(gates)
        int orange_gate_1 = 1;
        int orange_gate_2 = 1;
        
        double tmp_distance_blue = 10, tmp_distance_yellow = 10, tmp_distance_orange = 10;
        if(!cones_s.empty())
        {
            for (size_t i = 0; i < cones_s.size(); i++){
                if (cones_s[i].BLUE == 2){
                    tmp_distance_blue = distance(cones_s[i].position.x, cones_s[i].position.y, 0, 0);//Verificar se o carro começa em 0
                    if (tmp_distance_blue < dist_b){
                        dist_b = tmp_distance_blue;
                        blue_index = i;
                    }
                }
                if (cones_s[i].YELLOW == 1){
                    tmp_distance_yellow = distance(cones_s[i].position.x, cones_s[i].position.y, 0, 0);
                    if (tmp_distance_yellow < dist_y){
                        dist_y = tmp_distance_yellow;
                        yellow_index = i;
                    }
                }
                if (cones_s[i].ORANGE_SMALL == 3){
                    tmp_distance_orange = distance(cones_s[i].position.x, cones_s[i].position.y, 0, 0);
                    if (tmp_distance_orange < dist_o){
                        dist_o = tmp_distance_orange;
                        if(orange_gate_1 == -1 && orange_gate_2 == -1)
                        {
                            orange_gate_1 = i;

                        }else{
                            orange_gate_2 = i;
                        }
                        
                    }
                }
            }
            //Calcula o ponto medio dos cones mais proximos ao caroo
            if (blue_index != -1 && yellow_index != -1 && orange_gate_1 != -1 && orange_gate_2 != -1) 
            {
                RCLCPP_INFO(this->get_logger(), "Tou aqui");
                
            //  /  RCLCPP_INFO(this->get_logger(), "gate 1: %.2f %.2f \n gate2: %.2f %.2f",cones_s[orange_gate_1].position.x, 
            //     cones_s[orange_gate_1].position.y, cones_s[orange_gate_2].position.x, cones_s[orange_gate_2].position.y);

                map_localizer(msg,blue_index,yellow_index,orange_gate_1,orange_gate_2,&map);
                map_Localized = true;
            }
        }
    }
}


geometry_msgs::msg::PoseStamped skidpad_node::track_correction(geometry_msgs::msg::PoseStamped pose){
    auto cones = coneArray->cones; 
    std::pair<double, double> pose_pos = {pose.pose.position.x, pose.pose.position.y};
    int blue_index = -1, yellow_index = -1;

    //Encontra os cones azuis e amarelos mais perto do pose
    double tmp_distance_blue = 0, tmp_distance_yellow = 0;
    if(!cones.empty()){
        double dist_b = std::numeric_limits<double>::max();
        double dist_y = std::numeric_limits<double>::max();
        for (size_t i = 0; i < cones.size(); i++){
            if (cones[i].BLUE == 2){
                tmp_distance_blue = distance(cones[i].position.x, cones[i].position.y, pose_pos.first,pose_pos.second);
                if (tmp_distance_blue < dist_b){
                    dist_b = tmp_distance_blue;
                    blue_index = i;
                }
            }
            if (cones[i].YELLOW == 1){
                tmp_distance_yellow = distance(cones[i].position.x, cones[i].position.y, pose_pos.first, pose_pos.second);
                if (tmp_distance_yellow < dist_y){
                    dist_y = tmp_distance_yellow;
                    yellow_index = i;
                }
            }
        }

        //Faz o ponto medio dos cones
        std::pair<double, double> midPoint;
        midPoint.first  = (cones[blue_index].position.x + cones[yellow_index].position.x)/2;
        midPoint.second = (cones[blue_index].position.y + cones[yellow_index].position.y)/2;
        
        double poseMidpoint_distance =  distance(midPoint.first,midPoint.second,pose_pos.first,pose_pos.second);
        //Se a distancia for menos que x vai puxar (Levar em conta o tamanho do carro e a largura da track)
        double threshold = 1.50-middleCar;
        if(poseMidpoint_distance >= threshold){
            RCLCPP_INFO(this->get_logger(), 
            "Tá fora da pista\n"
            "cones Usados para essa conta:\n"
            "BLUE:  %.2f, %.2f \n"
            "YELLOW: %.2f, %.2f",
            cones[blue_index].position.x, cones[blue_index].position.y,
            cones[yellow_index].position.x, cones[yellow_index].position.y);

            double correction_x = midPoint.first - pose_pos.first;
            double correction_y = midPoint.second - pose_pos.second;

            double target_yaw = std::atan2(correction_y, correction_x);

            pose.pose.position.x = correction_x;
            pose.pose.position.y = correction_y;
            //pose.pose.orientation.
        }else{
            return pose;
        } 
    }
    return pose;
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<skidpad_node>());
    rclcpp::shutdown();
    return 0;
}