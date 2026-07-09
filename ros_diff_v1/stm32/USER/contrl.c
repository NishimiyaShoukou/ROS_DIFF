#include "contrl.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "tim.h"
#include "gpio.h"
#include "pid.h"
#include "move.h"
#include "comHandle.h"
float Angle_begin,Angle_Set;

IMUDataTypedef g_IMU = 
{
	.pitch = 0, .roll  = 0, .yaw   = 0,
	.aacx  = 0, .aacy  = 0, .aacz  = 0,
	.gyrox = 0, .gyroy = 0, .gyroz = 0,
	.temp  = 0,
};
chasis_data Chasis_Loop={
	
	 .Motor_get_speed[0]=0,
	 .Motor_get_speed[1]=0,
	 .Motor_set_speed[0]=0,
	 .Motor_set_speed[1]=0,
	 .Motor_out[0]=0,
	 .Motor_out[1]=0
};


void PID_EXECUTOR(void)
{
	 static int div1,div2;
	 static float angle_out=0;
	 div1++;
	 div2++;
	 if(div1>=10)
	 {
		  
		 updataSpeed();
		 
		 
		 Chasis_Loop.Motor_out[0]=PID_speed_contrl(&pid_spd[0],Chasis_Loop.Motor_set_speed[0],Chasis_Loop.Motor_get_speed[0]+angle_out);
		 Chasis_Loop.Motor_out[1]=PID_speed_contrl(&pid_spd[1],Chasis_Loop.Motor_set_speed[1],Chasis_Loop.Motor_get_speed[1]-angle_out);
//		 Chasis_Loop.Motor_out[0]=Incremental_PID(&pid_spd[0],Chasis_Loop.Motor_set_speed[0],Chasis_Loop.Motor_get_speed[0]+angle_out);
//		 Chasis_Loop.Motor_out[1]=Incremental_PID(&pid_spd[1],Chasis_Loop.Motor_set_speed[1],Chasis_Loop.Motor_get_speed[1]-angle_out);
//		 //Incremental_PID(pid_t *pid,float set,float get);
		 
		 Set_Pwm(Chasis_Loop.Motor_out[0],Chasis_Loop.Motor_out[1]);
		
		 div1=0;
	 }
	 if(div2>=20)
	 {
		// goto_1m();
		  //Usartdata_up((short)(Chasis_Loop.Motor_get_speed[0]/ 50*22.5),(short) (Chasis_Loop.Motor_get_speed[1]/ 50*22.5),(short)g_IMU.yaw,'1');
  		 Usartdata_up(Chasis_Loop.Motor_get_speed[0],Chasis_Loop.Motor_get_speed[1],g_IMU.yaw,'a');
		 
// 		 angle_out=PID_contrl(&pid_turn,Angle_Set,g_IMU.yaw);
		 Chasis_Loop.Motor_set_speed[0]=-Recdata.speed[0]/100;
		 Chasis_Loop.Motor_set_speed[1]=-Recdata.speed[1]/100;
//		 Chasis_Loop.Motor_set_speed[0]=10;
//		 Chasis_Loop.Motor_set_speed[1]=10;
		 div2=0;
	 }
	 

}


void Set_speed(float spd1,float spd2)
{
	 Chasis_Loop.Motor_set_speed[0]=-spd1;
	 Chasis_Loop.Motor_set_speed[1]=-spd2;
}


void Set_speed_back(float spd1,float spd2)
{
	
	 Chasis_Loop.Motor_set_speed[0]=-spd1;
	 Chasis_Loop.Motor_set_speed[1]=-spd2;
	
}
/********************************
 设定电机pwm
 ********************************/
 
void Set_Pwm(int16_t pwm1,int16_t pwm2)
{
	 if(pwm1>=0)
	 {
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
		  
		  TIM1->CCR1=pwm1;
	 }
	 else
	 {
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
		  
		  TIM1->CCR1=-pwm1;
		   
	 }
	 
	 	if(pwm2>=0)
	 {
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
		  
		  TIM1->CCR2=pwm2;
	 }
	 else
	 {
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
		  
		  TIM1->CCR2=-pwm2;
		   
	 }
	
}
//平台位姿数据更新
void ImuDataUpdate(void)
{
	//获取欧拉角
//	mpu_dmp_get_data(&(g_IMU.pitch),&(g_IMU.roll),&(g_IMU.yaw));
	if(mpu_dmp_get_data(&(g_IMU.pitch),&(g_IMU.roll),&(g_IMU.yaw))==0)
	{		
		g_IMU.temp = MPU_Get_Temperature();					//得到温度值
		MPU_Get_Accelerometer(&(g_IMU.aacx),&(g_IMU.aacy),&(g_IMU.aacz));	//得到加速度传感器数据
		MPU_Get_Gyroscope(&(g_IMU.gyrox),&(g_IMU.gyroy),&(g_IMU.gyroz));	//得到陀螺仪数据
	}
//	if(g_IMU.yaw<=-180)
//	{
//			g_IMU.yaw+=360;
//	}
//	else if(g_IMU.yaw>=180)
//	{
//			g_IMU.yaw-=360;
//				
//	}
	
	
}
//速度值更新，读取编码器转速(线速度需要乘以19cm)
void updataSpeed(void)
{
	int16_t etr[2];
	
	//ML1 TIM2 MR2 TIM3
	
	//获得编码器值
	etr[0] = -TIM2->CNT;//M1
	TIM2->CNT = 0;
	etr[1] = TIM3->CNT;//M2
	TIM3->CNT = 0;
	
	for(int i = 0;i < 2;i++)
	{
		float etrtemp = 0;
		
		//编码器脉冲计算轮角速度
		//ppr=13,1:30,13*30=390
		etrtemp = ((float)(etr[i] )) / (13 * 30);
		
		//10ms更新一次
		Chasis_Loop.Motor_get_speed[i]=etrtemp*1500;
		//角速度计算轮线速度
//		etrtemp = etrtemp * (75 * PI);
//		//写入线速度到全局
//		g_MotorSpeed[i] = etrtemp * 10.0 * 0.25;
	}
	//Chasis_Loop.positionNow +=  (Chasis_Loop.Motor_get_speed[0]*0.01 + Chasis_Loop.Motor_get_speed[1]*0.01 ) / 50*24;
}



