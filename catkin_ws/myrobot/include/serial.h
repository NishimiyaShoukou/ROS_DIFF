
  #ifndef _SERIAL_H_
#define _SERIAL_H_
#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"//use data struct of std_msgs/String  
#include "std_msgs/Float32.h" 
#include "turtlesim/Pose.h"  
#include "serial.h"
#include <vector>
//��Щͷ�ļ�������������Ҳ�����������ȫ��������
 
 
 
 
 void serialInit();
void writeSpeed(double Left_v, double Right_v,unsigned char ctrlFlag);
 bool readSpeed(double &Left_v,double &Right_v,double &Angle,unsigned char &ctrlFlag);
unsigned char getCrc8(unsigned char *ptr, unsigned short len);
 
void odom_pub_calcu(void);
double sin_cal(double theta);

#endif
