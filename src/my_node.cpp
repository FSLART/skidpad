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

    track_correction(&pathSpline_msg);


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
        auto original_map = map;

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
                map = original_map;
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

void skidpad_node::track_correction(lart_msgs::msg::PathSpline *path){
   if(!path || !coneArray){
    return; 
   }
   
    const auto& cones_s = coneArray->cones;
    auto original_path = *path;

    double soma_erro_x      = 0.0;
    double soma_erro_y      = 0.0;
    double pontos_validos   = 0.0;

    if(cones_s.empty() || path->poses.empty())
        return;

    if (cones_s.size() > 5000) {
        return;
    }

    //we only need the blue & yellow index
    //we are going to consider the first point of the path to translate
    for (size_t i = 0; i < 20; i++)
    {
        std::pair<double, double> pose_pos = {path->poses[i].pose.position.x, path->poses[i].pose.position.y}; 
        
        //locking for the nearst cone in relation to the pose
        double nearstCone_blue = -1;
        double nearstCone_yellow = -1;

        double blue_distnace = std::numeric_limits<double>::max();
        double yellow_distnace = std::numeric_limits<double>::max();
        
        for (size_t j = 0; j < cones_s.size(); j++)
        {
            

            double tmp_distance;
            if(cones_s[j].BLUE == lart_msgs::msg::Cone::BLUE){
                tmp_distance = distance(cones_s[j].position.x,cones_s[j].position.y,pose_pos.first,pose_pos.second);
                if (blue_distnace > tmp_distance)
                {
                    blue_distnace = tmp_distance;
                    nearstCone_blue = j;
                }
            }
            if (cones_s[j].YELLOW == lart_msgs::msg::Cone::YELLOW)
            {
                tmp_distance = distance(cones_s[j].position.x,cones_s[j].position.y,pose_pos.first,pose_pos.second);
                if (yellow_distnace > tmp_distance)
                {
                    yellow_distnace = tmp_distance;
                    nearstCone_yellow = j;
                }
            }
        }

        if (nearstCone_blue == -1 || nearstCone_yellow == -1) {
            continue;
        }

        // Validar a largura do par de cones (limite de 7.5m para lidar com as curvas)
        double pair_distance = distance(
            cones_s[nearstCone_blue].position.x, cones_s[nearstCone_blue].position.y,
            cones_s[nearstCone_yellow].position.x, cones_s[nearstCone_yellow].position.y
        );

        if (pair_distance > 7.5) {
            continue; 
        }

        //Midpoint cones
        double midPoint_x = (cones_s[nearstCone_blue].position.x + cones_s[nearstCone_yellow].position.x)/2;
        double midPoint_y = (cones_s[nearstCone_blue].position.y + cones_s[nearstCone_yellow].position.y)/2;

        //Calulating the error
        soma_erro_x += (midPoint_x - pose_pos.first);
        soma_erro_y += (midPoint_y - pose_pos.second);
        pontos_validos++;

    
    }

    // Se no fim do ciclo não houve nenhum ponto válido, não mexemos no path
    if (pontos_validos == 0) {
        return;
    }

    // --- FAZER AS MÉDIAS ---
    double erro_medio_x = soma_erro_x / pontos_validos;
    double erro_medio_y = soma_erro_y / pontos_validos;

    // --- FILTRAR O ERRO MÉDIO (EMA) ---
    const double ALPHA = 0.15; 
    double filtered_corr_x = ALPHA * erro_medio_x + (1.0 - ALPHA) * this->prev_corr_x_;
    double filtered_corr_y = ALPHA * erro_medio_y + (1.0 - ALPHA) * this->prev_corr_y_;

    // --- CLAMP (Proteção contra guinadas de 30cm) ---
    const double MAX_CORRECTION = 0.30; 
    double corr_magnitude = std::sqrt(filtered_corr_x * filtered_corr_x + filtered_corr_y * filtered_corr_y);

    if (corr_magnitude > MAX_CORRECTION) {
        double scale = MAX_CORRECTION / corr_magnitude;
        filtered_corr_x *= scale;
        filtered_corr_y *= scale;
    }

    // Guardar para o próximo ciclo do ROS
    this->prev_corr_x_ = filtered_corr_x;
    this->prev_corr_y_ = filtered_corr_y;

    // --- MOMENTO FINAL: DESLOCAR O PATH TODO ---
    for (auto& pt : path->poses)
    {
        pt.pose.position.x += filtered_corr_x;
        pt.pose.position.y += filtered_corr_y;
    }
}




int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<skidpad_node>());
    rclcpp::shutdown();
    return 0;
}