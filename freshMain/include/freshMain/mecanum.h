#ifndef _MECANUM_H_ 
#define _MECANUM_H_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "odometry.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <iomanip>

#define allowance 10e-3
#define NSS 10 //num_of_SoftStart
class Mecanum{
private:
    double maxLinearSpeed = 1;      //m/s
    double maxAngularSpeed = 0.5;     //rad/s
public:
    double softStart;
    double limitLinearSpeed, limitAngularSpeed;
    Odometry odometry;
    Mecanum(){
        softStart = 0;
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
        diff_theta = (des_theta/180*PI) - odometry.theta;    
        while(std::abs(diff_theta) > PI)   diff_theta = (diff_theta > 0)?diff_theta - 2*PI:diff_theta + 2*PI;
        speed.angular.z = (std::abs(diff_theta) > allowance)?std::abs(speed_Kp) * diff_theta:0;

        limitLinearSpeed = (softStart/maxLinearSpeed < NSS)?softStart/NSS:maxLinearSpeed;
        limitAngularSpeed = (softStart/maxAngularSpeed < 2*NSS)?softStart/2/NSS:maxAngularSpeed;
        
        if(std::abs(speed.linear.x) > limitLinearSpeed)  speed.linear.x = (speed.linear.x > 0)?limitLinearSpeed:-1*limitLinearSpeed;
        if(std::abs(speed.linear.y) > limitLinearSpeed)  speed.linear.y = (speed.linear.y > 0)?limitLinearSpeed:-1*limitLinearSpeed;
        if(std::abs(speed.angular.z) > limitAngularSpeed)  speed.angular.z = (speed.angular.z > 0)?limitAngularSpeed:-1*limitAngularSpeed;
        if(std::abs(diff_x) <= allowance && std::abs(diff_y) <= allowance && std::abs(diff_theta) <= allowance)    if_reach = true;
        
        std::cout<<softStart<<"\t";

        std::cout<<std::fixed<<std::setprecision(4);
        std::cout<<"des("<<des_x<<","<<des_y<<","<<des_theta<<")\t";
        std::cout<<"odo("<<odometry.x<<","<<odometry.y<<","<<odometry.theta*180/PI<<")\t";
        std::cout<<"vel("<<speed.linear.x<<","<<speed.linear.y<<","<<speed.angular.z<<")\n";

        return speed;
    }
};
int readPath(double* des_x_Ptr, double* des_y_Ptr, double* des_theta_Ptr, size_t& current_index);
#endif