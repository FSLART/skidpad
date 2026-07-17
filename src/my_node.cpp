#include "../include/my_node.hpp"

using std::placeholders::_1;

skidpad_node::skidpad_node() : Node("skidpadNode")
{
    RCLCPP_INFO(this->get_logger(), "Skidpad node has been started");

    this->path_vis_pub = this->create_publisher<nav_msgs::msg::Path>(TOPIC_PATH_MARKER, 10);
    this->path_control_pub = this->create_publisher<lart_msgs::msg::PathSpline>(TOPIC_PATH, 10);

    this->cone_array_subscriber = this->create_subscription<lart_msgs::msg::ConeArray>(TOPIC_CONES, 10, std::bind(&skidpad_node::coneArrayCallback, this, _1));
    this->position_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(TOPIC_SLAM_POSE, 10, std::bind(&skidpad_node::positionCallback, this, _1));
    this->rpm_subscriber = this->create_subscription<lart_msgs::msg::Dynamics>(TOPIC_CONTROL_FEEDBACK,10, std::bind(&skidpad_node::RpmCallback,this,_1));

    //mudar isto NAO PODE TER PATH ABSOLUTO ******************************
    map = file_loader("skidpad_path_xyk3.csv");
};

void skidpad_node::SplitLineSender()
{
    auto stamp = this->now();

    // 1. Inicialização das mensagens
    lart_msgs::msg::PathSpline pathSpline_msg;
    pathSpline_msg.header.stamp = stamp;
    pathSpline_msg.header.frame_id = "world";

    nav_msgs::msg::Path path_rviz_msg;
    path_rviz_msg.header.stamp = stamp;
    path_rviz_msg.header.frame_id = "world";

    if (map.empty())
        return; // Prevenção de segurança

    // 2. Encontrar o ponto de partida: o mais próximo do carro, com leve preferência
    // por continuidade de índice (evita saltar para a interseção do skidpad)
    double best_score = std::numeric_limits<double>::max();
    std::size_t closest_idx = last_idx_;

    for (std::size_t i = 0; i < map.size(); i++)
    {
        double dx = map[i].x - carData.car_x;
        double dy = map[i].y - carData.car_y;
        double dist = std::sqrt(dx * dx + dy * dy);

        std::size_t idx_diff = (i > last_idx_) ? (i - last_idx_) : (last_idx_ - i);
        idx_diff = std::min(idx_diff, map.size() - idx_diff); // distância circular

        double score = dist + idx_diff * 0.01;

        if (score < best_score)
        {
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
        carData.roll, carData.pitch, carData.yaw, stamp);
    pathSpline_msg.poses.push_back(pose);
    pathSpline_msg.curvature.push_back(map[start_idx].cur);
    pathSpline_msg.distance.push_back(cumulative_dist);
    path_rviz_msg.poses.push_back(pose);

    // 3. Iterar sequencialmente (usando while e módulo para lidar com o circuito fechado)
    std::size_t i = (start_idx + 1) % map.size(); // Começa no ponto a seguir
    std::size_t pontos_verificados = 0;           // Segurança contra loops infinitos

    while (pathSpline_msg.poses.size() < 100 && pontos_verificados < map.size())
    {

        // Distância ao ÚLTIMO ponto que enviámos
        double d = distance(last_added_x, last_added_y, map[i].x, map[i].y);

        // Só guarda se a distância for maior ou igual a 50cm
        if (d >= 0.1)
        {
            pose = createPoseMsg(
                map[i].x, map[i].y,
                carData.roll, carData.pitch, carData.yaw, stamp);
            // Verification zone
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

    track_correction(&pathSpline_msg, &path_rviz_msg);

    // 4. Publicar apenas UMA vez no final da função
    if (!pathSpline_msg.poses.empty())
    {
        // RCLCPP_INFO(this->get_logger(), "Enviado Path: %zu pontos, Distância total: %.2fm",
        //             pathSpline_msg.poses.size(), cumulative_dist);
        path_control_pub->publish(pathSpline_msg);
        path_vis_pub->publish(path_rviz_msg);
    }
}

// Sem localizar o mapa primeiro
void skidpad_node::positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    carData.car_x = msg->pose.position.x;
    carData.car_y = msg->pose.position.y;

    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);

    tf2::Matrix3x3 m(q);
    m.getRPY(carData.roll, carData.pitch, carData.yaw);

    if (map_Localized)
    {
        SplitLineSender();
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Map is not localized");
    }
}

void skidpad_node::coneArrayCallback(const lart_msgs::msg::ConeArray::SharedPtr msg)
{
    auto cones_s = msg->cones;
    skidpad_node::coneArray = msg;

    auto original_map = map;

    if (!map_Localized)
    {
        int blue_index = -1, yellow_index = -1;
        int orange_index_1 = -1, orange_index_2 = -1;
        nearest_cone(msg, &blue_index, &yellow_index, &orange_index_1, &orange_index_2);

        if (blue_index != -1 && yellow_index != -1 && orange_index_1 != -1 && orange_index_2 != -1)
        {
            // experimenta as duas ORDENS do gate — resolve heading invertido
            std::vector<std::pair<int,int>> gate_orders = {
                {orange_index_1, orange_index_2},
                {orange_index_2, orange_index_1}
            };

            for (auto& [g1, g2] : gate_orders)
            {
                std::vector<PathStruct> candidate = original_map;   // SEMPRE do template limpo
                map_localizer(msg, blue_index, yellow_index, g1, g2, &candidate);

                double s = score_map(candidate, msg);   // matching de cones, não depende da pose
                RCLCPP_INFO(this->get_logger(), "Score %.3f | melhor ate agora %.3f", s, best_map_distance);

                if (s < best_map_distance)
                {
                    best_map_distance = s;
                    map = candidate;
                }
            }
            map_trys++;
        }

        // decide se já está bom o suficiente para TRANCAR
        if (best_map_distance < LOCK_THRESHOLD || map_trys < MAP_LOCALIZER_TRYS)
        {
            map_Localized = true;
            RCLCPP_INFO(this->get_logger(), "SUCESSO: Mapa ancorado (%.3f)", best_map_distance);
        }
        return;
    }
}

// void skidpad_node::track_correction(lart_msgs::msg::PathSpline *path, nav_msgs::msg::Path *path_vis)
// {
//     //VARIAVEIS DE CONTROLO 
//     const size_t NUMERO_DE_PONTOS = 40;     // AFINAÇÃO 1:  ("Efeito Atalho")
//     const double PAIR_DISTANCE_CONTROL = 4; //Limite da distancia para ligar os cones para evitar ligar cones de uma ponta a outra na pista
//     const double ALPHA = 0.30;          //EMA FILTER Ex Aumentado para 0.40 para reagir mais rápido e puxar para o meio
//     const double MAX_CORRECTION = 0.6;    //Proteção contra guinadas  Ex: Aumentado para 1.50m para o carro ter liberdade de chegar ao meio
//     double lookAhead_dynamic = (0.5*carData.velocity) +2.0; // distancia a olhar para a frente para calcular a media  



//     if (!path || !coneArray)
//         return;
    
//     // PROTEÇÃO 2: Lock temporal e referência segura.
//     auto local_coneArray = coneArray;
//     const auto &cones_s = local_coneArray->cones;

//     // PROTEÇÃO 3: Cortar Loops Infinitos
//     if (cones_s.size() > 5000)
//         return;

//     // Proteção original: se não houver cones ou o path estiver vazio
//     if (cones_s.empty() || path->poses.empty())
//         return;

//     double soma_erro_x = 0.0;
//     double soma_erro_y = 0.0;
//     int pontos_validos = 0;

//     // Ponto de referência inicial (o carro) para calcular a distância dos 2 metros
//     double start_x = path->poses[0].pose.position.x;
//     double start_y = path->poses[0].pose.position.y;

//     size_t num_pontos = std::min((size_t)NUMERO_DE_PONTOS, path->poses.size());

//     for (size_t i = 0; i < num_pontos; i++)
//     {
//         double pt_x = path->poses[i].pose.position.x;
//         double pt_y = path->poses[i].pose.position.y;

//         // Se já olhamos mais de 2 metros para a frente, paramos de calcular a média!
//         if (distance(start_x, start_y, pt_x, pt_y) > lookAhead_dynamic)
//             break;

//         std::pair<double, double> pose_pos = {pt_x, pt_y};
        
//         int nearstCone_blue = -1;
//         int nearstCone_yellow = -1;
//         double blue_distnace = std::numeric_limits<double>::max();
//         double yellow_distnace = std::numeric_limits<double>::max();

//         for (size_t j = 0; j     < cones_s.size(); j++)
//         {
//             double tmp_distance;
//             if (cones_s[j].class_type.data == lart_msgs::msg::Cone::BLUE)
//             {
//                 tmp_distance = distance(cones_s[j].position.x, cones_s[j].position.y, pose_pos.first, pose_pos.second);
//                 if (blue_distnace > tmp_distance)
//                 {
//                     blue_distnace = tmp_distance;
//                     nearstCone_blue = j;
//                 }
//             }

//             if (cones_s[j].class_type.data == lart_msgs::msg::Cone::YELLOW){
//                 tmp_distance = distance(cones_s[j].position.x, cones_s[j].position.y, pose_pos.first, pose_pos.second);
//                 if (yellow_distnace > tmp_distance){
//                     yellow_distnace = tmp_distance;
//                     nearstCone_yellow = j;
//                 }
//             }
//         }

//         if (nearstCone_blue == -1 || nearstCone_yellow == -1){
//             continue;
//         }

//         // Validar a largura do par de cones (limite de 7.5m para lidar com as curvas)
//         double pair_distance = distance(
//             cones_s[nearstCone_blue].position.x, cones_s[nearstCone_blue].position.y,
//             cones_s[nearstCone_yellow].position.x, cones_s[nearstCone_yellow].position.y
//         );

//         if (pair_distance > PAIR_DISTANCE_CONTROL)
//             continue;
        

//         // Midpoint cones
//         double midPoint_x = (cones_s[nearstCone_blue].position.x + cones_s[nearstCone_yellow].position.x) / 2.0;
//         double midPoint_y = (cones_s[nearstCone_blue].position.y + cones_s[nearstCone_yellow].position.y) / 2.0;

//         // Calculating the error e Acumular
//         soma_erro_x += (midPoint_x - pt_x);
//         soma_erro_y += (midPoint_y - pt_y);
//         pontos_validos++;
//     }

//     // Se no fim do ciclo não houve nenhum ponto válido, não mexemos no path
//     if (pontos_validos == 0)
//     {
//         RCLCPP_WARN(this->get_logger(), "[SKIDPAD CORRECTION] 0 pontos validos. Path inalterado.");
//         return;
//     }

//     // --- FAZER AS MÉDIAS ---
//     double erro_medio_x = soma_erro_x / pontos_validos;
//     double erro_medio_y = soma_erro_y / pontos_validos;

//     // MSG DEBUG 1: Mostra o erro cru que a matemática detetou (antes de cortar)
//     RCLCPP_INFO(this->get_logger(), "[SKIDPAD CORRECTION] Pts: %d | Erro Real: (X: %.2f, Y: %.2f)",
//                 pontos_validos, erro_medio_x, erro_medio_y);

    
//     // --- FILTRAR O ERRO MÉDIO (EMA) ---
//     double filtered_corr_x = ALPHA * erro_medio_x + (1.0 - ALPHA) * this->prev_corr_x_;
//     double filtered_corr_y = ALPHA * erro_medio_y + (1.0 - ALPHA) * this->prev_corr_y_;

    
//     double corr_magnitude = std::sqrt(filtered_corr_x * filtered_corr_x + filtered_corr_y * filtered_corr_y);

//     if (corr_magnitude > MAX_CORRECTION)
//     {
//         double scale = MAX_CORRECTION / corr_magnitude;
//         filtered_corr_x *= scale;
//         filtered_corr_y *= scale;
//         RCLCPP_WARN(this->get_logger(), "[SKIDPAD CORRECTION] Limite MAX (%f) atingido!",MAX_CORRECTION);
//     }

//     // MSG DEBUG 2: Mostra o erro exato que vai ser aplicado ao Path
//     RCLCPP_INFO(this->get_logger(), "[SKIDPAD CORRECTION] Aplicado: (X: %.2f, Y: %.2f) | Filtro_Mag: %.2fm",
//                 filtered_corr_x, filtered_corr_y, corr_magnitude);

//     // Guardar para o próximo ciclo do ROS
//     this->prev_corr_x_ = filtered_corr_x;
//     this->prev_corr_y_ = filtered_corr_y;

//     // --- MOMENTO FINAL: DESLOCAR O PATH TODO ---
//     // for (auto &pt : path->poses)
//     // {
//     //     pt.pose.position.x += filtered_corr_x;
//     //     pt.pose.position.y += filtered_corr_y;
//     // }
//     //
//     // // TEMP COMENTAR DEPOIS
//     // for (auto &pt1 : path_vis->poses)
//     // {
//     //     pt1.pose.position.x += filtered_corr_x;
//     //     pt1.pose.position.y += filtered_corr_y;
//     // }

//     size_t total_poses = path->poses.size();
//     for (size_t k = 0; k < total_poses; k++)
//     {
//         // O peso começa em 1.0 (100%) no primeiro ponto e vai descendo até 0.0 (0%) no último
//         double decaimento = 1.0 - ((double)k / (double)total_poses);
        
//         path->poses[k].pose.position.x += (filtered_corr_x * decaimento);
//         path->poses[k].pose.position.y += (filtered_corr_y * decaimento);
       
//         path_vis->poses[k].pose.position.x += (filtered_corr_x * decaimento);
//         path_vis->poses[k].pose.position.y += (filtered_corr_y * decaimento);
//     }
// }
void skidpad_node::track_correction(lart_msgs::msg::PathSpline *path, nav_msgs::msg::Path *path_vis)
{
    // VARIÁVEIS DE CONTROLO
    const double PAIR_DISTANCE_CONTROL = 6.0;
    const double ALPHA = 0.70;                  // amortecimento aplicado só ao PAR_IDEAL
    const double MAX_CORRECTION = 1.2;          // limite para o modo PAR_IDEAL (puxar ao meio)
    const double MAX_CORRECTION_REPULSAO = 1.7; // limite maior para as repulsões de emergência
    double lookAhead_dynamic = (0.5 * carData.velocity) + 4.0;

    // DEBUG 1: Estado do Carro e da Janela de Visão
    RCLCPP_INFO(this->get_logger(), "[SKIDPAD] V: %.2fm/s | LookAhead: %.2fm", carData.velocity, lookAhead_dynamic);

    if (!path || !coneArray) return;
    
    auto local_coneArray = coneArray;
    const auto &cones_s = local_coneArray->cones;

    if (cones_s.size() > 5000 || cones_s.empty() || path->poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "[SKIDPAD] Abortado: Sem cones ou sem path.");
        return;
    }

    double start_x = path->poses[0].pose.position.x;
    double start_y = path->poses[0].pose.position.y;
    
    int pontos_corrigidos = 0; // Para o resumo final

    // Iteramos por todos os pontos do path
    for (size_t i = 0; i < path->poses.size(); i++)
    {
        double pt_x = path->poses[i].pose.position.x;
        double pt_y = path->poses[i].pose.position.y;

        if (distance(start_x, start_y, pt_x, pt_y) > lookAhead_dynamic) {
            // Se o primeiro ponto falhar o limite, escusamos de fazer prints para os restantes 30 pontos ignorados
            break;
        }

        std::pair<double, double> pose_pos = {pt_x, pt_y};
        
        int nearstCone_blue = -1;
        int nearstCone_yellow = -1;
        double blue_distance = std::numeric_limits<double>::max();
        double yellow_distance = std::numeric_limits<double>::max();

        // Encontrar os cones mais próximos
        for (size_t j = 0; j < cones_s.size(); j++)
        {
            double tmp_distance;
            if (cones_s[j].class_type.data == lart_msgs::msg::Cone::BLUE)
            {
                tmp_distance = distance(cones_s[j].position.x, cones_s[j].position.y, pose_pos.first, pose_pos.second);
                if (blue_distance > tmp_distance) {
                    blue_distance = tmp_distance;
                    nearstCone_blue = j;
                }
            }
            if (cones_s[j].class_type.data == lart_msgs::msg::Cone::YELLOW)
            {
                tmp_distance = distance(cones_s[j].position.x, cones_s[j].position.y, pose_pos.first, pose_pos.second);
                if (yellow_distance > tmp_distance) {
                    yellow_distance = tmp_distance;
                    nearstCone_yellow = j;
                }
            }
        }

        double erro_x = 0.0;
        double erro_y = 0.0;
        double pair_distance = std::numeric_limits<double>::max();
        std::string acao = "IGNORADO"; // Regista o que o algoritmo decidiu fazer

        if (nearstCone_blue != -1 && nearstCone_yellow != -1) {
            pair_distance = distance(
                cones_s[nearstCone_blue].position.x, cones_s[nearstCone_blue].position.y,
                cones_s[nearstCone_yellow].position.x, cones_s[nearstCone_yellow].position.y
            );
        }

        // SE O PAR FOR VÁLIDO: Lógica Normal
        if (nearstCone_blue != -1 && nearstCone_yellow != -1 && pair_distance <= PAIR_DISTANCE_CONTROL) 
        {
            double midPoint_x = (cones_s[nearstCone_blue].position.x + cones_s[nearstCone_yellow].position.x) / 2.0;
            double midPoint_y = (cones_s[nearstCone_blue].position.y + cones_s[nearstCone_yellow].position.y) / 2.0;
            
            erro_x = midPoint_x - pt_x;
            erro_y = midPoint_y - pt_y;
            acao = "PAR_IDEAL";
        }
        // FALLBACKS: Repulsão do Amarelo e/ou do Azul.
        // Avaliados de forma INDEPENDENTE (não em cadeia else-if) para que um cone
        // perto de um lado nunca "roube a vez" à repulsão do outro lado.
        else
        {
            const double CLEARANCE_DESEJADA = 1.5;
            bool aplicou_amarelo = false;
            bool aplicou_azul = false;

            if (nearstCone_yellow != -1 && yellow_distance < 2.5)
            {
                double dir_x = pt_x - cones_s[nearstCone_yellow].position.x;
                double dir_y = pt_y - cones_s[nearstCone_yellow].position.y;
                double mag = yellow_distance;

                // Só empurra para fora se estivermos mais perto do que a clearance desejada.
                // Nunca puxar para o cone: se mag >= CLEARANCE_DESEJADA o push fica a zero.
                double push = std::max(0.0, CLEARANCE_DESEJADA - mag);
                if (mag > 0.0 && push > 0.0) {
                    erro_x += (dir_x / mag) * push * 1.5;
                    erro_y += (dir_y / mag) * push * 1.5;
                    aplicou_amarelo = true;
                }
            }

            if (nearstCone_blue != -1 && blue_distance < 2.5)
            {
                double dir_x = pt_x - cones_s[nearstCone_blue].position.x;
                double dir_y = pt_y - cones_s[nearstCone_blue].position.y;
                double mag = blue_distance;

                // Só empurra para fora se estivermos mais perto do que a clearance desejada.
                // Nunca puxar para o cone: se mag >= CLEARANCE_DESEJADA o push fica a zero.
                double push = std::max(0.0, CLEARANCE_DESEJADA - mag);
                if (mag > 0.0 && push > 0.0) {
                    erro_x += (dir_x / mag) * push * 1.5;
                    erro_y += (dir_y / mag) * push * 1.5;
                    aplicou_azul = true;
                }
            }

            if (aplicou_amarelo && aplicou_azul) {
                acao = "REPULSAO_AMBOS";
            } else if (aplicou_amarelo) {
                acao = "REPULSAO_AMARELO";
            } else if (aplicou_azul) {
                acao = "REPULSAO_AZUL";
            } else {
                continue;
            }
        }

        pontos_corrigidos++;

        bool is_repulsao = (acao == "REPULSAO_AMARELO" || acao == "REPULSAO_AZUL" || acao == "REPULSAO_AMBOS");
        double limite_ativo = is_repulsao ? MAX_CORRECTION_REPULSAO : MAX_CORRECTION;
        // Repulsão de emergência: aplica o erro por inteiro (o "push" já foi calculado como
        // a distância exata que falta para a clearance; amortecer devolvia o ponto para perto do cone).
        // PAR_IDEAL: amortecido por ALPHA, como pedido.
        double peso_aplicacao = is_repulsao ? 1.0 : ALPHA;

        double erro_magnitude = std::sqrt(erro_x * erro_x + erro_y * erro_y);

        // Proteção contra guinadas
        if (erro_magnitude > limite_ativo) {
            double scale = limite_ativo / erro_magnitude;
            erro_x *= scale;
            erro_y *= scale;

            // DEBUG 2: Quando o limite é ativado num ponto específico
            RCLCPP_WARN(this->get_logger(), "[SKIDPAD] Pt %zu (%s) | CORTE: Erro %.2f reduzido para %.2f",
                        i, acao.c_str(), erro_magnitude, limite_ativo);
        } else {
            // DEBUG 3: Comportamento normal por ponto
            RCLCPP_INFO(this->get_logger(), "[SKIDPAD] Pt %zu (%s) | Aplicado: %.2fm",
                        i, acao.c_str(), erro_magnitude);
        }

        path->poses[i].pose.position.x += erro_x * peso_aplicacao;
        path->poses[i].pose.position.y += erro_y * peso_aplicacao;

        if (path_vis && i < path_vis->poses.size()) {
            path_vis->poses[i].pose.position.x += erro_x * peso_aplicacao;
            path_vis->poses[i].pose.position.y += erro_y * peso_aplicacao;
        }
    }

    // DEBUG 4: Resumo do ciclo
    RCLCPP_INFO(this->get_logger(), "[SKIDPAD] Fim de ciclo. %d pontos corrigidos dentro do LookAhead.", pontos_corrigidos);
}
void skidpad_node::RpmCallback(const lart_msgs::msg::Dynamics msg){
    carData.rpm = msg.rpm;
    carData.velocity = RPM_TO_MS(msg.rpm);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<skidpad_node>());
    rclcpp::shutdown();
    return 0;
}