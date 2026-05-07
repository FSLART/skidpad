#include "../include/utils.hpp"

double distance(double x, double y, double x1, double y1){
    return (double)std::sqrt((x1 - x) * (x1 - x) + (y1 - y) * (y1 - y));
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