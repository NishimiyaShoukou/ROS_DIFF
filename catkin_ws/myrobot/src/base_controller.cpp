/* base_controller.cpp */
#include "ros/ros.h"  //ros��Ҫ��ͷ�ļ�
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
float D = 0.14;    //dis of two wheel m
float linear_temp = 9, angular_temp = 9;//�ݴ�����ٶȺͽ��ٶ�
float reductionSpeedRatio = 26.666; //���ٱ�
/****************************************************/
string rec_buffer;  //�������ݽ��ձ���

float left_speed = 0, right_speed = 0, left_speed_temp, right_speed_temp;
unsigned char control_flag=0x07;

//test receive value
double left_speed_now=0.0;
double right_speed_now=0.0;
double angle=0.0;
double last_angle=0.0;
unsigned char testRece4=0x00;
/************************************************************/
void callback(const geometry_msgs::Twist& cmd_input)//����/cmd_vel����ص�����
{
	
	cout << "linear_temp: " << linear_temp << "    " << "angular_temp: " << angular_temp << endl;

	linear_temp = cmd_input.linear.x;//��ȡ/cmd_vel�����ٶ�.m/s
	angular_temp = cmd_input.angular.z;//��ȡ/cmd_vel�Ľ��ٶ�,rad/s
	
	cout << "linear_temp: " << linear_temp << "    " << "angular_temp: " << angular_temp << endl;
		

	left_speed = (linear_temp - 0.5f * angular_temp * D) * 1000; //mm/s
	right_speed = (linear_temp + 0.5f * angular_temp * D) * 1000; //mm/s
	
	

	left_speed_temp = left_speed* reductionSpeedRatio ; //r/min
	right_speed_temp =right_speed * reductionSpeedRatio ;

	

	
    writeSpeed(left_speed_temp,right_speed_temp,control_flag);




}

int main(int argc, char** argv)
{
	

	ros::init(argc, argv, "base_controller");//
	ros::NodeHandle n;  //����ڵ���̾��

	ros::Subscriber sub = n.subscribe("cmd_vel", 20, callback); //����/cmd_vel����
	ros::NodeHandle nh;  //����ڵ���̾��
    ros::Publisher pub = nh.advertise<myrobot::speed>("/car_state",10);
	//4 ��֯������Ϣ
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

	    readSpeed(left_speed_now,right_speed_now,angle,testRece4);
        double dt = (current_time - last_time).toSec();
       //writeSpeed(-10,-10,control_flag);
		p.vx=0.5*0.225*(right_speed_now+left_speed_now)/60;
     
        //p.w=(right_speed_now-left_speed_now)/D;
		p.w=((angle-last_angle)*0.01745329/88)/dt;
		pub.publish(p);
		last_angle=angle;
		last_time =current_time ;
        //��ӡ����
		ROS_INFO("we receive 1 %f,%f,%f,%f\n",p.vx,p.w,angle,dt);
	    ROS_INFO("we receive 2 %f,%f,%f,%d\n",left_speed_now,right_speed_now,angle,testRece4);
 
        loop_rate.sleep();
    }
    return 0;
}


