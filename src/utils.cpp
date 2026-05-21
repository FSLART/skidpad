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
void map_localizer(const lart_msgs::msg::ConeArray::SharedPtr msg, int blue_index,int yellow_index, std::vector<PathStruct> *map){
    auto cones_s = msg->cones;
    std::vector<PathStruct> temp_map = *map;

    //calcula o ponto medio
    double bx = cones_s[blue_index].position.x;
    double by = cones_s[blue_index].position.y;
    double yx = cones_s[yellow_index].position.x;
    double yy = cones_s[yellow_index].position.y;

    double midpoint_x = (bx+yx)/2;
    double midpoint_y = (by+yy)/2;

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

        // path.x = original_x * cos_tr - path.y * sin_tr+tx;
        // path.y = original_x * sin_tr + path.y * cos_tr+ty;

        path.x = (original_x + midpoint_x) * cos_tr - (path.y + midpoint_y) * sin_tr + tx;
        path.y = (original_x + midpoint_x) * sin_tr + (path.y + midpoint_y) * cos_tr + ty;
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