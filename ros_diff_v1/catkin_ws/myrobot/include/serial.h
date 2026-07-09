#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "turtlesim/Pose.h"
#include "serial.h"
#include <vector>

// 串口初始化
void serialInit(void);

// 写速度函数，用于发送给下位机控制器
void writeSpeed(double Left_v, double Right_v, unsigned char ctrlFlag);

// 读速度函数
bool readSpeed(double& Left_v, double& Right_v, double& Angle, unsigned char& ctrlFlag);

// CRC8 校验
unsigned char getCrc8(unsigned char* ptr, unsigned short len);

void odom_pub_calcu(void);
double sin_cal(double theta);

#endif