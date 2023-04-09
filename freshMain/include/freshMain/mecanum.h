#ifndef _MECANUM_H_ 
#define _MECANUM_H_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "odometry.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <yaml-cpp/yaml.h>

#define allowance 10e-3
class Mecanum{
public:
    Odometry odometry;
    Mecanum(){
        odometry.x = 0;
        odometry.y = 0;
        odometry.theta = 0;
    }
    bool if_reach = false;
    geometry_msgs::Twist goTo(double des_x, double des_y, double des_theta, double speed_Kp){
        geometry_msgs::Twist speed;
        double diff_x, diff_y, diff_theta;
        
        diff_x = des_x - odometry.x;    speed.linear.x = (std::abs(diff_x) > allowance)?std::abs(speed_Kp) * diff_x:0;
        diff_y = des_y - odometry.y;    speed.linear.y = (std::abs(diff_y) > allowance)?std::abs(speed_Kp) * diff_y:0;
        diff_theta = des_theta - odometry.theta;    
        while(std::abs(diff_theta) > PI)   diff_theta = (diff_theta > 0)?diff_theta - 2*PI:diff_theta + 2*PI;
        speed.angular.z = (std::abs(diff_theta) > allowance)?std::abs(speed_Kp) * diff_theta:0;
        if(std::abs(diff_x) <= allowance && std::abs(diff_y) <= allowance && std::abs(diff_theta) <= allowance)    if_reach = true;
        
        std::cout
        <<"x:( "
        <<des_x<<" "
        <<odometry.x<<" "
        <<speed.linear.x<<" )\t"
        <<"y:( "
        <<des_y<<" "
        <<odometry.y<<" "
        <<speed.linear.y<<" )\t"
        <<"theta:( "
        <<des_theta<<" "
        <<odometry.theta<<" "
        <<speed.angular.z<<" )\n";

        return speed;
    }
};
int readPath(double* des_x_Ptr, double* des_y_Ptr, double* des_theta_Ptr);
#endif