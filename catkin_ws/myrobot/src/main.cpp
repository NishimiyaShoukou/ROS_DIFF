 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial.h"
 
//test send value
double left_speed_ctrl=-20.0;
double right_speed_ctrl=-20.0;
unsigned char control_flag=0x07;
 
//test receive value
double left_speed_now=0.0;
double right_speed_now=0.0;
double angle=0.0;
unsigned char testRece4=0x00;
 
int main(int agrc,char **argv)
{
    ros::init(agrc,argv,"node");
    ros::NodeHandle nh;
 
    ros::Rate loop_rate(50);
    
    //���ڳ�ʼ��
    serialInit();
  ROS_INFO("we start\n");
    while(ros::ok())
    {
        ros::spinOnce();
        //��STM32�˷������ݣ�ǰ����Ϊdouble���ͣ����һ��Ϊunsigned char����
	    writeSpeed(left_speed_ctrl,right_speed_ctrl,control_flag);
         ROS_INFO("we start2\n");
        //��STM32�������ݣ������������ת��ΪС�������ٶȡ����ٶȡ�����ǣ��Ƕȣ���Ԥ������λ
	    readSpeed(left_speed_now,right_speed_now,angle,testRece4);
        //��ӡ����
	    ROS_INFO("we receive %f,%f,%f,%d\n",left_speed_now,right_speed_now,angle,testRece4);
 
        loop_rate.sleep();
    }
    return 0;
}
 
 
 
 