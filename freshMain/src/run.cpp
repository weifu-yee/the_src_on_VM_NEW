#include "cmath"
#include "iostream"
#include "sys/time.h"

#include "odometry.h"
#include "mecanum.h"

double speed_Kp, des_x, des_y, des_theta;
double des_x_last = -1, des_y_last = -1, des_theta_last = -1;

Odometry odometry;
Mecanum mecanum;

void Callback(const geometry_msgs::Twist::ConstPtr& ins_vel){
    mecanum.odometry.update(ins_vel);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    // ros::Subscriber pose_sub = nh.subscribe("/ins_vel",10,Callback);
    ros::Subscriber fake_odometry = nh.subscribe("/cmd_vel",1,Callback);

    ros::Rate r(10);
    
    while(ros::ok()){
        nh.getParam("/speed",speed_Kp);
        nh.getParam("/des_x",des_x);
        nh.getParam("/des_y",des_y);
        nh.getParam("/des_theta",des_theta);
        if(des_x_last != des_x || des_y_last != des_y || des_theta_last != des_theta){
            while(!mecanum.if_reach && ros::ok()){
                ros::spinOnce();
                vel_pub.publish( mecanum.goTo(des_x, des_y, des_theta, speed_Kp) );
                // readPath("../params/points_array.yaml");
            }
            mecanum.if_reach = false;
            std::cout<<"\n\t\tarrive the destanation!\n\n";
        }
        des_x_last = des_x;
        des_y_last = des_y;
        des_theta_last = des_theta;
    }
    return 0;
}
