#include "../include/my_node.hpp"

using std::placeholders::_1;

skidpad_node::skidpad_node() : Node("skidpadNode")
{
    RCLCPP_INFO(this->get_logger(), "Skidpad node has been started");

    this->path_vis_pub = this->create_publisher<nav_msgs::msg::Path>(TOPIC_PATH_MARKER, 10);
    this->path_control_pub = this->create_publisher<lart_msgs::msg::PathSpline>(TOPIC_PATH, 10);

    this->cone_array_subscriber = this->create_subscription<lart_msgs::msg::ConeArray>(TOPIC_CONES, 10, std::bind(&skidpad_node::coneArrayCallback, this, _1));
    this->position_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(TOPIC_SLAM_POSE, 10, std::bind(&skidpad_node::positionCallback, this, _1));
    map = file_loader("skidpad_path_xyk.csv");
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

   track_correction(&pathSpline_msg,&path_rviz_msg);


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

void skidpad_node::track_correction(lart_msgs::msg::PathSpline *path, nav_msgs::msg::Path *path_vis){
  // PROTEÇÃO 1: Evitar ler ponteiros nulos
    if (!path || !coneArray) {
        return;
    auto local_coneArray = coneArray;
    const auto& cones_s = local_coneArray->cones;
    
    if (cones_s.size() > 5000 || cones_s.empty() || path->poses.empty()) 
        return;

    double soma_erro_x      = 0.0;
    double soma_erro_y      = 0.0;
    int pontos_validos      = 0;

    double start_x = path->poses[0].pose.position.x;
    double start_y = path->poses[0].pose.position.y;

    // AFINAÇÃO 1: Reduzido de 20 para 8 para não cortar a curva ("Efeito Atalho")
    size_t num_pontos = std::min((size_t)14, path->poses.size());

    for (size_t i = 0; i < num_pontos; i++)
    {
        double pt_x = path->poses[i].pose.position.x;
        double pt_y = path->poses[i].pose.position.y;
        
        // Se já olhamos mais de 2 metros para a frente, paramos de calcular a média!
        if (distance(start_x, start_y, pt_x, pt_y) > 2) {
            break;

        // 1. Descobrir a orientação do path neste ponto (Heading / Theta)
        double theta = 0.0;
        if (i + 1 < path->poses.size()) {
            double nx = path->poses[i+1].pose.position.x;
            double ny = path->poses[i+1].pose.position.y;
            theta = std::atan2(ny - pt_y, nx - pt_x);
        } else if (i > 0) {
            double px = path->poses[i-1].pose.position.x;
            double py = path->poses[i-1].pose.position.y;
            theta = std::atan2(pt_y - py, pt_x - px);
        }

        double cos_theta = std::cos(theta);
        double sin_theta = std::sin(theta);
        
        int best_blue_idx = -1;
        int best_yellow_idx = -1;
        double min_score = std::numeric_limits<double>::max();

        // 2. Procurar cones AZUIS (Obrigatoriamente à ESQUERDA)
        for (size_t b = 0; b < cones_s.size(); b++) {
            if (cones_s[b].BLUE != lart_msgs::msg::Cone::BLUE) 
                continue;
            
            double dx = cones_s[b].position.x - pt_x;
            double dy = cones_s[b].position.y - pt_y;
            
            // Matemática de Referencial Local (Forward = X, Left = Y)
            double local_x = dx * cos_theta + dy * sin_theta;
            double local_y = -dx * sin_theta + dy * cos_theta;
            
            // Filtro rigoroso: Tem de estar no raio de 3m à frente/trás, e Pelo menos 0.3m à ESQUERDA (Y Positivo)
            if (std::abs(local_x) > 3.0 || local_y < 0.3) 
                continue;

            double dist_to_blue = std::sqrt(dx*dx + dy*dy);

            // 3. Procurar cones AMARELOS (Obrigatoriamente à DIREITA)
            for (size_t y = 0; y < cones_s.size(); y++) {
                if (cones_s[y].YELLOW != lart_msgs::msg::Cone::YELLOW) 
                    continue;
                
                double dx_y = cones_s[y].position.x - pt_x;
                double dy_y = cones_s[y].position.y - pt_y;
                
                double local_x_y = dx_y * cos_theta + dy_y * sin_theta;
                double local_y_y = -dx_y * sin_theta + dy_y * cos_theta;
                
                // Filtro rigoroso: Pelo menos 0.3m à DIREITA (Y Negativo)
                if (std::abs(local_x_y) > 3.0 || local_y_y > -0.3) 
                    continue;

                double dist_to_yellow = std::sqrt(dx_y*dx_y + dy_y*dy_y);
                
                double gate_width = distance(cones_s[b].position.x, cones_s[b].position.y, 
                                           cones_s[y].position.x, cones_s[y].position.y);
                
                if (gate_width < 2.0 || gate_width > 5.0) 
                    continue;

                // Score: Preferir cones alinhados
                double score = (dist_to_blue + dist_to_yellow) + std::abs(dist_to_blue - dist_to_yellow) * 1.5;

                if (score < min_score) {
                    min_score = score;
                    best_blue_idx = b;
                    best_yellow_idx = y;
                }
            }
        }

        if (best_blue_idx == -1 || best_yellow_idx == -1) 
            continue;

        double midPoint_x = (cones_s[best_blue_idx].position.x + cones_s[best_yellow_idx].position.x) / 2.0;
        double midPoint_y = (cones_s[best_blue_idx].position.y + cones_s[best_yellow_idx].position.y) / 2.0;

        //evita que os cones azul e amarelo estejam longe e se conectem
        if (pair_distance > 5) {
            continue; 
        }

        // Midpoint cones 
        double midPoint_x = (cones_s[nearstCone_blue].position.x + cones_s[nearstCone_yellow].position.x) / 2.0;
        double midPoint_y = (cones_s[nearstCone_blue].position.y + cones_s[nearstCone_yellow].position.y) / 2.0;

        // Calculating the error e Acumular
        soma_erro_x += (midPoint_x - pt_x);
        soma_erro_y += (midPoint_y - pt_y);
        pontos_validos++;
    }

    if (pontos_validos == 0) 
        return;

    double erro_medio_x = soma_erro_x / pontos_validos;
    double erro_medio_y = soma_erro_y / pontos_validos;

    const double MAX_CORRECTION = 0.40; 
    double corr_magnitude = std::sqrt(erro_medio_x * erro_medio_x + erro_medio_y * erro_medio_y);

    // --- FILTRAR O ERRO MÉDIO (EMA) ---
    // AFINAÇÃO 2: Aumentado para 0.40 para reagir mais rápido e puxar para o meio
    const double ALPHA = 0.30; 
    double filtered_corr_x = ALPHA * erro_medio_x + (1.0 - ALPHA) * this->prev_corr_x_;
    double filtered_corr_y = ALPHA * erro_medio_y + (1.0 - ALPHA) * this->prev_corr_y_;

    // --- CLAMP (Proteção contra guinadas) ---
    // AFINAÇÃO 3: Aumentado para 1.50m para o carro ter liberdade de chegar ao meio
    const double MAX_CORRECTION = 1.5; 
    double corr_magnitude = std::sqrt(filtered_corr_x * filtered_corr_x + filtered_corr_y * filtered_corr_y);

    if (corr_magnitude > MAX_CORRECTION) {
        double scale = MAX_CORRECTION / corr_magnitude;
        filtered_corr_x *= scale;
        filtered_corr_y *= scale;
        RCLCPP_WARN(this->get_logger(), "[SKIDPAD CORRECTION] Limite MAX (%f) atingido!",MAX_CORRECTION);
    }

    for (auto& pt : path->poses)
    {
        pt.pose.position.x += final_corr_x;
        pt.pose.position.y += final_corr_y;
    }
    //TEMP COMENTAR DEPOIS
    for (auto& pt : path_vis->poses)
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