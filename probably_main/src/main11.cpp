#include "ros/ros.h"
#include "odometry.h"

#define allowance 10e-3

double x = 0, y = 0, theta = 0, dt;
ros::Time current_time, last_time ;
bool if_reach = false;
double speed_Kp, des_x, des_y, des_theta;
double des_x_last = -1, des_y_last = -1, des_theta_last = -1;

void Callback(const geometry_msgs::Twist::ConstPtr& ins_vel){
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();
    if(dt > 1)  dt = 0;
    whereAmI(&x,&y,&theta,ins_vel,dt);
    last_time = current_time;
}

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
    ros::init(argc, argv, "ver0326_3vel");
    ros::NodeHandle nh;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // ros::Subscriber pose_sub = nh.subscribe("/ins_vel",10,Callback);
    ros::Subscriber fake_odometry = nh.subscribe("/cmd_vel",10,Callback);

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
                vel_pub.publish( GoToPoint(des_x, des_y, des_theta, speed_Kp) );
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
