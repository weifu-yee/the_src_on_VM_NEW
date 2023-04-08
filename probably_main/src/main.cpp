#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point32.h"
#include "cmath"
#include "iostream"
#include "sys/time.h"

#define PI 3.1415926
#define allowance 10e-3

// Global variables to store position and orientation
bool if_reach = false;
double speed_Kp, des_x, des_y, des_theta;
double des_x_last = -1, des_y_last = -1, des_theta_last = -1;

class odometry{
private:
    ros::Time current_time, last_time;
    double dt;
public:
    double x, y, theta;
    odometry(){
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
odometry odometry;

class mecanum{
private:

public:
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
mecanum mecanum;

// Callback function to update position and orientation
void Callback(const geometry_msgs::Twist::ConstPtr& ins_vel){
    odometry.update(ins_vel);
}

// Function to compute the desired speed to reach the target point
geometry_msgs::Twist GoToPoint(double des_x, double des_y, double des_theta, double speed_Kp){
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

int main(int argc, char **argv){
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    // ros::Subscriber pose_sub = nh.subscribe("/ins_vel",10,Callback);
    ros::Subscriber fake_odometry = nh.subscribe("/cmd_vel",1,Callback);

    ros::Rate r(10);
    
    // Main loop to continuously check and update target position and orientation

    while(ros::ok()){
        nh.getParam("/speed",speed_Kp);
        nh.getParam("/des_x",des_x);
        nh.getParam("/des_y",des_y);
        nh.getParam("/des_theta",des_theta);
        if(des_x_last != des_x || des_y_last != des_y || des_theta_last != des_theta){
            while(!if_reach && ros::ok()){
                ros::spinOnce();
                vel_pub.publish( mecanum.goTo(des_x, des_y, des_theta, speed_Kp) );
            }
            if_reach = false;
            std::cout<<"\n\t\tarrive the destanation!\n\n";
        }
        des_x_last = des_x;
        des_y_last = des_y;
        des_theta_last = des_theta;
    }
    return 0;
}
