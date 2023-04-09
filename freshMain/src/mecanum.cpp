#include "mecanum.h"

int readPath(double* des_x_Ptr, double* des_y_Ptr, double* des_theta_Ptr, size_t& current_index){
    std::ifstream file("/home/wave/catkin_ws/src/freshMain/params/points_array.yaml");
    if (!file.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return 1;
    }

    YAML::Node pathConfig = YAML::Load(file);
    if (current_index >= pathConfig.size()) {
        std::cout<<"over\n";
        return 1;
    }

    int i = -1;
    for (auto twistElement : pathConfig) {
        auto twist = twistElement["twist"];
        *des_x_Ptr = twist[0].as<double>();
        *des_y_Ptr = twist[1].as<double>();
        *des_theta_Ptr = twist[2].as<double>();
        if(++i == current_index)  break;
    }
    current_index ++;
    return 0;
}