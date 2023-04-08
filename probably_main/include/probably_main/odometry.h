#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

void whereAmI(double* x, double* y, double* theta, const geometry_msgs::Twist::ConstPtr& ins_vel, double dt);

#endif