/**
 * @file base_controller.cpp
 * @author thoelc
 * @brief 实现对stm32底盘控制功能
 * @version 1.0
 * @date 2023-11-06
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "ros/ros.h"  
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <string>        
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include "serial.h"
#include<fcntl.h>
#include<stdio.h>   
#include<string.h>
#include "myrobot/speed.h"
using namespace std;
/*****************************************************************************/
float D = 0.14;    // dis of two wheel m
float linear_temp = 9, angular_temp = 9;// 存储获得线速度和角速度值
float reductionSpeedRatio = 26.666; //转换成轮子转速
/****************************************************/


float left_speed = 0, right_speed = 0, left_speed_temp, right_speed_temp;
unsigned char control_flag=0x07;

// 当前从下位机读取到左右轮速度和底盘角度值(差速底盘)
double left_speed_now=0.0;
double right_speed_now=0.0;
double angle=0.0;
double last_angle=0.0;
unsigned char testRece4=0x00;
/************************************************************/
void callback(const geometry_msgs::Twist& cmd_input)// cmd_vel话题接收回调函数
{
	
	cout << "linear_temp: " << linear_temp << "    " << "angular_temp: " << angular_temp << endl;

	linear_temp = cmd_input.linear.x;// 当有cmd_vel话题更新时更新速度x方向
	angular_temp = cmd_input.angular.z;//锟斤拷取/cmd_vel锟侥斤拷锟劫讹拷,rad/s
	
	cout << "linear_temp: " << linear_temp << "    " << "angular_temp: " << angular_temp << endl;
		

	left_speed = (linear_temp - 0.5f * angular_temp * D) * 1000; //mm/s
	right_speed = (linear_temp + 0.5f * angular_temp * D) * 1000; //mm/s
	
	

	left_speed_temp = left_speed* reductionSpeedRatio ; //r/min
	right_speed_temp = right_speed * reductionSpeedRatio ;

	

	
    writeSpeed(left_speed_temp,right_speed_temp,control_flag);




}

int main(int argc, char** argv)
{
	

	ros::init(argc, argv, "base_controller");//
	ros::NodeHandle n;  //创建句柄

	ros::Subscriber sub = n.subscribe("cmd_vel", 20, callback); //订阅/cmd_vel话题
	ros::NodeHandle nh;  
    ros::Publisher pub = nh.advertise<myrobot::speed>("/car_state",10);  // 发布小车速度信息/car_state话题
	//4 锟斤拷织锟斤拷锟斤拷锟斤拷息
	myrobot::speed p;


    ros::Rate loop_rate(50);
    ros::Time current_time, last_time;

    current_time = ros::Time::now();

    last_time = ros::Time::now();
    
    serialInit();
    ROS_INFO("we start\n");
    while(ros::ok())
    {
        ros::spinOnce();
        current_time = ros::Time::now();
		// 读取速度值和角度值
	    readSpeed(left_speed_now,right_speed_now,angle,testRece4);
		// 获取单位时间
        double dt = (current_time - last_time).toSec();
       //writeSpeed(-10,-10,control_flag);
	   //进行速度合成 取两轮速度平均值作为小车前进速度
		p.vx=0.5*0.225*(right_speed_now+left_speed_now)/60;
		// 目前选用读取小车角度变化进行微分获得角速度，也可以根据两轮速度计算单误差更大
        //p.w=(right_speed_now-left_speed_now)/D;
		p.w=((angle-last_angle)*0.01745329/88)/dt;
		pub.publish(p);
		last_angle=angle;
		last_time =current_time ;
        // 测试输出可以删掉
		ROS_INFO("we receive 1 %f,%f,%f,%f\n",p.vx,p.w,angle,dt);
	    ROS_INFO("we receive 2 %f,%f,%f,%d\n",left_speed_now,right_speed_now,angle,testRece4);
 
        loop_rate.sleep();
    }
    return 0;
}


