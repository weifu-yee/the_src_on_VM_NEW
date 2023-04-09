#include "mecanum.h"

int readPath(double* des_x_Ptr, double* des_y_Ptr, double* des_theta_Ptr){
    std::ifstream file("/home/wave/catkin_ws/src/freshMain/params/points_array.yaml");
    if (!file.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return 1;
    }

    YAML::Node pathConfig = YAML::Load(file);

    for (auto twistElement : pathConfig) {
        auto twist = twistElement["twist"];
        *des_x_Ptr = twist[0].as<double>();
        *des_y_Ptr = twist[1].as<double>();
        *des_theta_Ptr = twist[2].as<double>();
        // std::cout<<*des_x<<"\t"<<*des_y<<"\t"<<*des_theta<<"\n";

        // MECANUM::moveTo(x, y, z);
    }

    return 0;
}

// int readPath(std::string yaml_path){
//     std::ifstream file("/home/wave/catkin_ws/src/freshMain/params/points_array.yaml");
//     // std::cout << "Attempting to open file: " << yaml_path << std::endl;
//     if (!file.is_open()) {
//         std::cerr << "Error opening file!" << std::endl;
//         return 1;
//     }

//     YAML::Node pathConfg = YAML::Load(yaml_path);
//     std::vector<double> twistArray = pathConfg["twist"].as<std::vector<double>>();

//     double x,y,theta;
//     for(auto twist : pathConfg){
//         x = twist["twist"][0].as<double>();
//         y = twist["twist"][1].as<double>();
//         theta = twist["twist"][2].as<double>();

//         std::cout<<"\n\n"<<x<<"\t"<<y<<"\t"<<theta<<"\n";

//         // MECANUM::moveTo(x,y,z);
//     }
//     return 0;
// }