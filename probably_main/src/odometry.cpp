#include "odometry.h"
#define PI 3.1415926

void whereAmI(double* x, double* y, double* theta, const geometry_msgs::Twist::ConstPtr& ins_vel, double dt){
    *x += ins_vel->linear.x * (dt);
    *y += ins_vel->linear.y * (dt);
    *theta += ins_vel->angular.z * (dt);

    while(*theta > PI)  *theta -= 2*PI;
    while(*theta < -PI) *theta += 2*PI;
    return;
}