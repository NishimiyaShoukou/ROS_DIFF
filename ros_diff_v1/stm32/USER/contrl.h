#ifndef __CONTRL_H__
#define __CONTRL_H__
#include "stm32f4xx.h"


typedef struct 
{
	 //motor contrl
	 float Motor_get_speed[2];
	 float Motor_set_speed[2];
	 double positionNow;
	 float Motor_out[2];
	
	 //angle contrl
	 
	 
}chasis_data;

typedef struct
{
	float pitch,roll,yaw; 		//欧拉角
	short aacx,aacy,aacz;			//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
	short temp;
}IMUDataTypedef;

extern IMUDataTypedef g_IMU;
extern float Angle_begin,Angle_Set;
void Set_Pwm(int16_t pwm1,int16_t pwm2);
void updataSpeed(void);
void PID_EXECUTOR(void);
void ImuDataUpdate(void);
void Set_speed(float spd1,float spd2);
void Set_speed_back(float spd1,float spd2);
extern chasis_data Chasis_Loop;
#endif

