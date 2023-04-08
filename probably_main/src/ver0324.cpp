#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point32.h"
#include "cmath"
#include "iostream"
#include "sys/time.h"

#define PI 3.1415926
#define allowance 10e-3 // 裕度值

double x = 0, y = 0, dt; // 機器人當前位置及時間間隔
ros::Time current_time, last_time ; // 當前時間和上一次時間
int step = 0, step_last = -1; // 步驟變量及上一步驟變量
bool if_reach = false; // 是否到達目標變量
double speed_Kp, des_x, des_y; // 速度控制增益、目標x坐標、目標y坐標
double des_x_last = -1, des_y_last = -1; // 上一次目標x坐標、上一次目標y坐標

void Callback(const geometry_msgs::Twist::ConstPtr& ins_vel){
    current_time = ros::Time::now(); // 當前時間
    dt = (current_time - last_time).toSec(); // 計算時間差

    if(dt > 1)  dt = 0; // 時間差過大，設為0

    x += ins_vel->linear.x * (dt); // 更新x坐標
    y += ins_vel->linear.y * (dt); // 更新y坐標
    
    last_time = current_time; // 上一次時間更新為當前時間
}

geometry_msgs::Twist GoToPoint(double destination_x, double destination_y, double speed_Kp){
    geometry_msgs::Twist speed; // 速度訊息
    double diff, ang;
    if(step == 0){       //x方向
        diff = destination_x - x; // 計算x方向差距
        speed.linear.x = abs(speed_Kp) * diff; // 根據控制增益計算x方向速度
        if(std::abs(diff) <= allowance)   step = 1; // 差距小於裕度，進入y方向
    }
    if(step == 1){       //y方向
        diff = destination_y - y; // 計算y方向差距
        speed.linear.y = abs(speed_Kp) * diff; // 根據控制增益計算y方向速度
        if(std::abs(diff) <= allowance){
            step = 0;
            if_reach = true; // 差距小於裕度，到達目標
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

    step_last = step; // 更新步驟變量
    return speed; // 返回速度訊息
}
int main(int argc, char **argv){
    ros::init(argc, argv, "ver0324"); // 初始化ROS节点
    ros::NodeHandle nh; // 创建节点句柄
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10); // 创建一个发布者，用于发布速度信息
    ros::Subscriber pose_sub = nh.subscribe("/ins_vel",10,Callback); // 创建一个订阅者，用于订阅当前位置信息
    // ros::Subscriber pose_sub = nh.subscribe("/cmd_vel",10,Callback); // fake

    ros::Rate r(10); // 设置循环频率为10Hz
    
    while(ros::ok()){ // 当ROS系统处于运行状态时执行循环
        nh.getParam("/speed",speed_Kp); // 获取速度Kp的参数值
        nh.getParam("/des_x",des_x); // 获取目标x坐标的参数值
        nh.getParam("/des_y",des_y); // 获取目标y坐标的参数值
        if(des_x_last != des_x || des_y_last != des_y){ // 如果目标坐标发生了变化
            while(!if_reach && ros::ok()){ // 当没有到达目标位置并且ROS系统仍在运行时执行循环
                ros::spinOnce(); // 处理一次回调函数
                vel_pub.publish( GoToPoint(des_x, des_y, speed_Kp) ); // 向/cmd_vel话题发布速度信息
            }
            step = 0, step_last = -1; // 将步数重置
            if_reach = false; // 标记没有到达目标位置
            std::cout<<"\n\t\tarrive the destanation!\n\n"; // 输出提示信息
        }
        des_x_last = des_x; // 保存上一次的目标x坐标
        des_y_last = des_y; // 保存上一次的目标y坐标
    }
    return 0; // 返回程序结束标志
}
