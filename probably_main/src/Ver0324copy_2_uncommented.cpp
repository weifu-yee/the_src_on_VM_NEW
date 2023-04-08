//這個備份已經去掉了x_sign，但沒有讓chatGPT寫註解
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point32.h"
#include "cmath"
#include "iostream"
#include "sys/time.h"

#define PI 3.1415926
#define allowance 10e-3

double x = 0, y = 0, dt;
ros::Time current_time, last_time ;
int step = 0, step_last = -1;
bool if_reach = false;
double speed_Kp, des_x, des_y;
double des_x_last = -1, des_y_last = -1;

void Callback(const geometry_msgs::Twist::ConstPtr& ins_vel){
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();

    if(dt > 1)  dt = 0;

    x += ins_vel->linear.x * (dt);
    y += ins_vel->linear.y * (dt);
    
    last_time = current_time;
}

geometry_msgs::Twist GoToPoint(double destination_x, double destination_y, double speed_Kp){
    geometry_msgs::Twist speed;
    double diff, ang;
    if(step == 0){       //x-direction
        diff = destination_x - x;
        speed.linear.x = abs(speed_Kp) * diff;
        if(std::abs(diff) <= allowance)   step = 1;
    }
    if(step == 1){       //y-direction
        diff = destination_y - y;
        speed.linear.y = abs(speed_Kp) * diff;
        if(std::abs(diff) <= allowance){
            step = 0;
            if_reach = true;
        }
    }

    // std::cout
    // <<"x:( "
    // <<destination_x<<"\t"
    // <<x<<" )\t"
    // <<"y:( "
    // <<destination_y<<"\t"
    // <<y<<" )\t"
    // <<diff<<"\n";

    step_last = step;
    return speed;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "ver0324");
    ros::NodeHandle nh;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber pose_sub = nh.subscribe("/ins_vel",10,Callback);

    // ros::Subscriber pose_sub = nh.subscribe("/cmd_vel",10,Callback);//fake
    
    ros::Rate r(10);
    
    while(ros::ok()){
        nh.getParam("/speed",speed_Kp);
        nh.getParam("/des_x",des_x);
        nh.getParam("/des_y",des_y);
        if(des_x_last != des_x || des_y_last != des_y){
            while(!if_reach && ros::ok()){
                ros::spinOnce();
                vel_pub.publish( GoToPoint(des_x, des_y, speed_Kp) );
                // r.sleep();
            }
            step = 0, step_last = -1;
            if_reach = false;
            std::cout<<"\n\t\tarrive the destanation!\n\n";
        }
        des_x_last = des_x;
        des_y_last = des_y;
    }
    return 0;
}