/**
 * @file main.cpp
 * @author Thoelc
 * @brief 机器人底盘控制测试程序,实现简单下位机底盘速度控制接收功能
 * @version 1.0
 * @date 2023-11-06
 * 
 * @copyright Copyright (c) Thoelc 2023
 * 
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial.h"
 
// test send value
double left_speed_ctrl=-20.0;
double right_speed_ctrl=-20.0;
unsigned char control_flag=0x07;
 
// test receive value
double left_speed_now=0.0;
double right_speed_now=0.0;
double angle=0.0;
unsigned char testRece4=0x00;
 
int main(int agrc,char **argv)
{
    ros::init(agrc,argv,"node");
    ros::NodeHandle nh;
    // 设置执行频率50hz，也就是大约20ms跑一次命令（不精准)
    ros::Rate loop_rate(50);
    
    // 串口初始化，也可以使用Rosserial实现
    serialInit();
    ROS_INFO("we start\n");
    while(ros::ok())
    {
        ros::spinOnce();
        // 发送速度命令给底盘
	      writeSpeed(left_speed_ctrl, right_speed_ctrl, control_flag);
         ROS_INFO("we start2\n");
        // 从STM32下位机读取速度和角度值
	      readSpeed(left_speed_now, right_speed_now, angle, testRece4);
        // 打印输出值
	      ROS_INFO("we receive %f,%f,%f,%d\n",left_speed_now,right_speed_now,angle,testRece4);
 
        loop_rate.sleep();
    }
    return 0;
}
 
 
 
 