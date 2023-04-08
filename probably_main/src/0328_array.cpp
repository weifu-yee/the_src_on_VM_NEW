#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point32.h"
#include "cmath"
#include "iostream"
#include "sys/time.h"
#include <vector>

#define PI 3.1415926
#define allowance 10e-3

// Global variables to store position and orientation
double x = 0, y = 0, theta = 0, dt;
ros::Time current_time, last_time ;
bool if_reach = false;
double speed_Kp, des_x, des_y, des_theta;
double des_x_last = -1, des_y_last = -1, des_theta_last = -1;

// Callback function to update position and orientation
void Callback(const geometry_msgs::Twist::ConstPtr& ins_vel){
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();

    if(dt > 1)  dt = 0;

    x += ins_vel->linear.x * (dt);
    y += ins_vel->linear.y * (dt);
    theta += ins_vel->angular.z * (dt);

    while(theta > PI)  theta -= 2*PI;
    while(theta < -PI) theta += 2*PI;
    
    last_time = current_time;
}

// Function to compute the desired speed to reach the target point
geometry_msgs::Twist GoToPoint(double des_x, double des_y, double des_theta, double speed_Kp){
    geometry_msgs::Twist speed;
    double diff_x, diff_y, diff_theta;
    
    diff_x = des_x - x;    speed.linear.x = (std::abs(diff_x) > allowance)?std::abs(speed_Kp) * diff_x:0;
    diff_y = des_y - y;    speed.linear.y = (std::abs(diff_y) > allowance)?std::abs(speed_Kp) * diff_y:0;
    diff_theta = des_theta - theta;    
    while(std::abs(diff_theta) > PI)   diff_theta = (diff_theta > 0)?diff_theta - 2*PI:diff_theta + 2*PI;
    speed.angular.z = (std::abs(diff_theta) > allowance)?std::abs(speed_Kp) * diff_theta:0;
    if(std::abs(diff_x) <= allowance && std::abs(diff_y) <= allowance && std::abs(diff_theta) <= allowance)    if_reach = true;
    
    std::cout
    <<"x:( "
    <<des_x<<" "
    <<x<<" "
    <<speed.linear.x<<" )\t"
    <<"y:( "
    <<des_y<<" "
    <<y<<" "
    <<speed.linear.y<<" )\t"
    <<"theta:( "
    <<des_theta<<" "
    <<theta<<" "
    <<speed.angular.z<<" )\n";

    return speed;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "0328_array");
    ros::NodeHandle nh;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber pose_sub = nh.subscribe("/ins_vel",10,Callback);
    // ros::Subscriber fake_odometry = nh.subscribe("/cmd_vel",10,Callback);

    ros::Rate r(10);

    // std::vector<double> des_x_array[10], des_y_array[10], des_theta_array[10];
    // nh.getParam("/speed",speed_Kp);
    // nh.getParam("/des_x",(double*) des_x_array);
    // nh.getParam("/des_y",(double*) des_y_array);
    // nh.getParam("/des_theta",(double*) des_theta_array);
    
    int t = 0;
    // Main loop to continuously check and update target position and orientation
    while(ros::ok()){
        // des_x = des_x_array[t];
        // des_y = des_y_array[t];
        // des_theta = des_theta_array[t];
        if(des_x_last != des_x || des_y_last != des_y || des_theta_last != des_theta){
            while(!if_reach && ros::ok()){
                ros::spinOnce();
                vel_pub.publish( GoToPoint(des_x, des_y, des_theta, speed_Kp) );
            }
            if_reach = false;
            // std::cout<<"\n\t\tarrive the destanation!\n\n";
        }
        des_x_last = des_x;
        des_y_last = des_y;
        des_theta_last = des_theta;
        t ++;
    }
    return 0;
}
