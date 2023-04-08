#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#define PI 3.1415926

class Odometry{
private:
    ros::Time current_time, last_time;
    double dt;
public:
    double x, y, theta;
    Odometry(){
        x = 0;
        y = 0;
        theta = 0;
    }
    void update(const geometry_msgs::Twist::ConstPtr& ins_vel){
        current_time = ros::Time::now();
        dt = (current_time - last_time).toSec();
        if(std::abs(dt) > 1)  dt = 0;
        x += ins_vel->linear.x * (dt);
        y += ins_vel->linear.y * (dt);
        theta += ins_vel->angular.z * (dt);
        while(theta > PI)  theta -= 2*PI;        
        while(theta < -PI) theta += 2*PI;
        last_time = current_time;
    }
};

#endif