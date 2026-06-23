#include "../include/utils.hpp"

double distance(double x, double y, double x1, double y1){
    return (double)std::sqrt((x1 - x) * (x1 - x) + (y1 - y) * (y1 - y));
}

geometry_msgs::msg::PoseStamped createPoseMsg(
    double x, double y, 
    double roll, double pitch, double yaw,
    const rclcpp::Time& stamp)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;

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

//rotação n tá a rodar bem as cenas 
//TEORIA: O PATH N TÀ NO MEIO DOS CONES DE FORMA PERFEITA 
void map_localizer(const lart_msgs::msg::ConeArray::SharedPtr msg, int blue_index,int yellow_index,int gate_index1, int gate_index2, std::vector<PathStruct> *map){
    auto cones_s = msg->cones;
    std::vector<PathStruct> temp_map = *map;

    //calcula o ponto medio
    double bx = cones_s[blue_index].position.x;
    double by = cones_s[blue_index].position.y;
    double yx = cones_s[yellow_index].position.x;
    double yy = cones_s[yellow_index].position.y;

    double gate1_x = cones_s[gate_index1].position.x;
    double gate1_y = cones_s[gate_index1].position.y;

    double gate2_x = cones_s[gate_index2].position.x;
    double gate2_y = cones_s[gate_index2].position.y;


    double midpoint_x = 0;//(gate1_x+gate2_x)/2;
    double midpoint_y = (gate1_y+gate2_y)/2;

    double tx = (bx + yx) / 2.0;
    double ty = (by + yy) / 2.0;

    //Calcular a Rotação Inicial (tr)
    // Na Formula Student: Azul é Esquerda, Amarelo é Direita.
    // O vetor vai do Azul para o Amarelo. Queremos a perpendicular (frente).
    double tr = atan2(yy - by, yx - bx); 

    double cos_tr = std::cos(tr);
    double sin_tr = std::sin(tr);

    for(PathStruct& path : temp_map){
        double original_x = path.x;

        path.x = (original_x + midpoint_x) * cos_tr - (path.y - midpoint_y) * sin_tr + tx;
        path.y = (original_x + midpoint_x) * sin_tr + (path.y - midpoint_y) * cos_tr + ty;
    }

    *map = temp_map;
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


//Return values are passed through pointers 
void nearest_cone(const lart_msgs::msg::ConeArray::SharedPtr msg, int *blue_index_o, int *yellow_index_o, int *orange_gate_1_o, int *orange_gate_2_o){
    if (blue_index_o)     *blue_index_o = -1;
    if (yellow_index_o)   *yellow_index_o = -1;
    if (orange_gate_1_o)  *orange_gate_1_o = -1;
    if (orange_gate_2_o)  *orange_gate_2_o = -1;


    auto cones_s = msg->cones;
    if(cones_s.empty()) {
        return;
    }

    double dist_b = std::numeric_limits<double>::max();
    double dist_y = std::numeric_limits<double>::max();
    
    // Precisamos de duas distâncias para os dois cones laranja mais próximos
    double dist_o1 = std::numeric_limits<double>::max();
    double dist_o2 = std::numeric_limits<double>::max();

    for (size_t i = 0; i < cones_s.size(); i++){
        
        // Azul (Normalmente a cor/id é um Enum ou Int, assume-se que .color == 2 ou similar, mantive a tua lógica)
        if (cones_s[i].BLUE == 2){
            double tmp_dist = distance(cones_s[i].position.x, cones_s[i].position.y, 0, 0);
            if (tmp_dist < dist_b){
                dist_b = tmp_dist;
                *blue_index_o = i;
            }
        }
        
        // Amarelo
        if (cones_s[i].YELLOW == 1){
            double tmp_dist = distance(cones_s[i].position.x, cones_s[i].position.y, 0, 0);
            if (tmp_dist < dist_y){
                dist_y = tmp_dist;
                *yellow_index_o = i;
            }
        }
        
        // Laranja
        if (cones_s[i].ORANGE_SMALL == 3){
            double tmp_dist = distance(cones_s[i].position.x, cones_s[i].position.y, 0, 0);
            
            if (tmp_dist < dist_o1) {
                // O antigo 1º mais próximo passa a ser o 2º mais próximo
                dist_o2 = dist_o1;
                *orange_gate_2_o = *orange_gate_1_o;
                
                // Este passa a ser o novo 1º mais próximo
                dist_o1 = tmp_dist;
                *orange_gate_1_o = i;
            } 
            else if (tmp_dist < dist_o2) {
                // É mais distante que o 1º, mas mais perto que o 2º atual
                dist_o2 = tmp_dist;
                *orange_gate_2_o = i;
            }
        }
    }
}