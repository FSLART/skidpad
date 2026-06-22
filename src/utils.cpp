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
void nearest_cone(const lart_msgs::msg::ConeArray::SharedPtr msg, int *blue_index_o, int *yellow_index_o, int *orange_index_o, int *orange_gate_1_o, int *orange_gate_2_o){
    auto cones_s = msg->cones;

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
                    *blue_index_o = i;
                }
            }
            if (cones_s[i].YELLOW == 1){
                tmp_distance_yellow = distance(cones_s[i].position.x, cones_s[i].position.y, 0, 0);
                if (tmp_distance_yellow < dist_y){
                    dist_y = tmp_distance_yellow;
                    *yellow_index_o = i;
                }
            }
            if (cones_s[i].ORANGE_SMALL == 3){
                tmp_distance_orange = distance(cones_s[i].position.x, cones_s[i].position.y, 0, 0);
                if (tmp_distance_orange < dist_o){
                    dist_o = tmp_distance_orange;
                    if(orange_gate_1 == -1 && orange_gate_2 == -1)
                    {
                        *orange_gate_1_o = i;
                    }else{
                        *orange_gate_2_o = i;
                    }
                }
            }
        }
    }else{
        return;
    }
}