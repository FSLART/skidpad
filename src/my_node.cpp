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
    //double total_dist = 0;
};

void skidpad_node::SplitLineSender(){
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
    // int start_idx = -1;
    // for(std::size_t i = 0; i < map.size(); i++){
    //     double dx = map[i].x - carData.car_x;
    //     double dy = map[i].y - carData.car_y;
        
    //     // Produto escalar simples para saber se o ponto está à frente
    //     double forward = dx * std::cos(carData.yaw) + dy * std::sin(carData.yaw);

    //     if(forward > 0.0){
    //         start_idx = i;
    //         break; 
    //     }
    // }

    // if(start_idx == -1) {
    //     RCLCPP_WARN(this->get_logger(), "Nenhum ponto do mapa encontrado à frente do carro!");
    //     return; 
    // }

    // membro da classe, inicializado a 0 (ou -1 na primeira chamada)
// std::size_t last_idx = 0;
// 2. Encontrar o ponto de partida: o mais próximo do carro, com leve preferência
    // por continuidade de índice (evita saltar para a interseção do skidpad)
    double best_score = std::numeric_limits<double>::max();
    std::size_t closest_idx = last_idx_;

    for(std::size_t i = 0; i < map.size(); i++){
        double dx = map[i].x - carData.car_x;
        double dy = map[i].y - carData.car_y;
        double dist = std::sqrt(dx*dx + dy*dy);

        std::size_t idx_diff = (i > last_idx_) ? (i - last_idx_) : (last_idx_ - i);
        idx_diff = std::min(idx_diff, map.size() - idx_diff); // distância circular

        double score = dist + idx_diff * 0.01;

        if(score < best_score){
            best_score = score;
            closest_idx = i;
        }
    }

    int start_idx = static_cast<int>(closest_idx);
    last_idx_ = start_idx; // guardar para a próxima iteração


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

    while(pathSpline_msg.poses.size() < 100 && pontos_verificados < map.size()){
        
        // Distância ao ÚLTIMO ponto que enviámos
        double d = distance(last_added_x, last_added_y, map[i].x, map[i].y);

        // Só guarda se a distância for maior ou igual a 50cm
        if(d >= 0.1){
            pose = createPoseMsg(
                map[i].x, map[i].y,
                carData.roll, carData.pitch, carData.yaw, stamp
            );
            //Verification zone
            //pose = skidpad_node::track_correction(pose);
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
    //RCLCPP_INFO(this->get_logger(), "Tou TOU TOU AQUI");

    auto cones_s = msg->cones;
    skidpad_node::coneArray = msg;

    if(!map_Localized)
    {
        RCLCPP_INFO(this->get_logger(), "Trying to localize the car");
        int blue_index = -1;
        int yellow_index = -1;
        int orange_index_1 = -1;
        int orange_index_2 = -1;

        nearest_cone(msg,&blue_index,&yellow_index,&orange_index_1,&orange_index_2);

        //Calcula o ponto medio dos cones mais proximos ao caroo
        if (blue_index != -1 && yellow_index != -1 && orange_index_1 != -1 && orange_index_2 != -1) 
        {
            double threshold_distance = 2;
            RCLCPP_INFO(this->get_logger(), "Tou aqui");

            //*-------------------------------------------*
            //*   MUDAR ISTO QUE TÀ HARDCODED NOS CONES   *
            //*   VERIFICAR E TESTAR SE O REATRY TA BOM   *
            //*-------------------------------------------*
            map_localizer(msg,blue_index,yellow_index,0,1,&map);
            RCLCPP_INFO(this->get_logger(), "PRIMEIRO PONTO DO MAP: (%.2f,%.2f) ",map[0].x,map[0].y);
            double realDisance = distance(carData.car_x,carData.car_y,map[0].x,map[0].y);
            if(realDisance > threshold_distance){
                RCLCPP_INFO(this->get_logger(), "A tentar de novo");

                map_localizer(msg,blue_index,yellow_index,0,1,&map);
                RCLCPP_INFO(this->get_logger(), "PRIMEIRO PONTO DO MAP: (%.2f,%.2f) ",map[0].x,map[0].y);
            }
            map_Localized = true;
            //RCLCPP_INFO(this->get_logger(), "PRIMEIRO PONTO DO MAP: (%.2f,%.2f) ",map[0].x,map[0].y);
            return;

        }
    }
}

geometry_msgs::msg::PoseStamped skidpad_node::track_correction(
    geometry_msgs::msg::PoseStamped pose)
{
    auto cones = coneArray->cones;
    if(cones.empty()) return pose;

    std::pair<double,double> pose_pos = {pose.pose.position.x, pose.pose.position.y};
    
    int blue_index = -1;
    int yellow_index = -1;
    double dist_b = std::numeric_limits<double>::max();
    double dist_y = std::numeric_limits<double>::max();

    for(size_t i = 0; i < cones.size(); i++){
        if(cones[i].BLUE == 2){
            double d = distance(cones[i].position.x, cones[i].position.y, pose_pos.first, pose_pos.second);
            if(d < dist_b){
                dist_b = d;
                blue_index = i;
            }
        }
        if(cones[i].YELLOW == 1){
            double d = distance(cones[i].position.x, cones[i].position.y, pose_pos.first, pose_pos.second);
            if(d < dist_y){
                dist_y = d;
                yellow_index = i;
            }
        }
    }

    if(blue_index == -1 || yellow_index == -1) return pose;

    // Midpoint correto: 0.5, não 0.25
    double mid_x = (cones[blue_index].position.x + cones[yellow_index].position.x) * 0.5;
    double mid_y = (cones[blue_index].position.y + cones[yellow_index].position.y) * 0.5;

    double correction_x = mid_x - pose_pos.first;
    double correction_y = mid_y - pose_pos.second;
    double error = std::sqrt(correction_x * correction_x + correction_y * correction_y);

    // Threshold mais baixo e ganho adaptativo (maior erro = maior correção)
    double threshold = 0.3;  // antes corrigir, mesmo com erro pequeno
    if(error < threshold) return pose;

    // Ganho dinâmico: proporcional ao erro, com limite máximo
    double gain = std::min(0.5, error * 0.2);  // até 50% do erro por frame, máx
    
    pose.pose.position.x += gain * correction_x;
    pose.pose.position.y += gain * correction_y;

    RCLCPP_INFO(this->get_logger(), 
        "Track correction | error=%.3f | gain=%.2f | corr=(%.3f, %.3f)",
        error, gain, correction_x, correction_y);

    return pose;
}

//Antiga
// geometry_msgs::msg::PoseStamped skidpad_node::track_correction(
//     geometry_msgs::msg::PoseStamped pose)
// {
//     auto cones = coneArray->cones;

//     if(cones.empty())
//         return pose;

//     std::pair<double,double> pose_pos = {
//         pose.pose.position.x,
//         pose.pose.position.y
//     };

//     int blue_index = -1;
//     int yellow_index = -1;

//     double dist_b = std::numeric_limits<double>::max();
//     double dist_y = std::numeric_limits<double>::max();

//     for(size_t i = 0; i < cones.size(); i++)
//     {
//         if(cones[i].BLUE == 2)
//         {
//             double d = distance(
//                 cones[i].position.x,
//                 cones[i].position.y,
//                 pose_pos.first,
//                 pose_pos.second);

//             if(d < dist_b)
//             {
//                 dist_b = d;
//                 blue_index = i;
//             }
//         }

//         if(cones[i].YELLOW == 1)
//         {
//             double d = distance(
//                 cones[i].position.x,
//                 cones[i].position.y,
//                 pose_pos.first,
//                 pose_pos.second);

//             if(d < dist_y)
//             {
//                 dist_y = d;
//                 yellow_index = i;
//             }
//         }
//     }

//     if(blue_index == -1 || yellow_index == -1)
//         return pose;

//     // midpoint entre os cones mais próximos
//     double mid_x =
//         (cones[blue_index].position.x +
//          cones[yellow_index].position.x) * 0.25;

//     double mid_y =
//         (cones[blue_index].position.y +
//          cones[yellow_index].position.y) * 0.25;

//     double correction_x = mid_x - pose_pos.first;
//     double correction_y = mid_y - pose_pos.second;

//     double error =
//         std::sqrt(correction_x * correction_x +
//                   correction_y * correction_y);

//     double threshold = 1.50 - middleCar;

//     if(error < threshold)
//         return pose;

//     // ganho pequeno para evitar saltos
//     const double gain = 0.20;

//     pose.pose.position.x += gain * correction_x;
//     pose.pose.position.y += gain * correction_y;

//     // RCLCPP_INFO(
//     //     this->get_logger(),
//     //     "Track correction | error=%.3f | corr=(%.3f, %.3f)",
//     //     error,
//     //     correction_x,
//     //     correction_y);

//     return pose;
// }


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<skidpad_node>());
    rclcpp::shutdown();
    return 0;
}