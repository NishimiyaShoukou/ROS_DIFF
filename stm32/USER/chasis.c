#include "chasis.h"
#include "stm32f4xx.h"
#include "tim.h"
#include "usart.h"
#include "pid.h"
#include "contrl.h"

#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#include "stdlib.h"
#include "stdio.h"

#include "move.h"


CarStateTypedef Miracle = WAIT;  //机器人状态机

void Main_Setup(void)
{
//	 uint8_t str[50];
	 static uint8_t temp = 1,i=0;
	 

	 //tim set
	 HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	 HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	 HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	 HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	
	
	 temp = mpu_dmp_init();
	 while(temp)
		{
			temp = mpu_dmp_init();
			
			printf("error\n\r");
			i++;
			if(i>10)
			{
				 break;
			}
			HAL_Delay(50);
		}
			
	 HAL_Delay(100);
	 //usaat set
	 __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
	 __HAL_UART_DISABLE_IT(&huart1,UART_IT_IDLE);
	 __HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
	 __HAL_UART_DISABLE_IT(&huart2,UART_IT_IDLE);
	
	 //pid set 
	 PID_Reset(&pid_spd[0],12.0f,1.0f,0.0f);
	 PID_Reset(&pid_spd[1],12.0f,1.0f,0.0f);
	 PID_Reset(&pid_turn,4.0f,0.0f,0.1f);
	 PID_Reset(&pid_move,2.0f,0.0f,2.0f);
	 	 	 //IMU Init
	
		
//		HAL_Delay(100);
//		if(mpu_dmp_get_data(&(g_IMU.pitch),&(g_IMU.roll),&(g_IMU.yaw))==0)
//		{		
//			g_IMU.temp = MPU_Get_Temperature();					//得到温度值
//			MPU_Get_Accelerometer(&(g_IMU.aacx),&(g_IMU.aacy),&(g_IMU.aacz));	//得到加速度传感器数据
//			MPU_Get_Gyroscope(&(g_IMU.gyrox),&(g_IMU.gyroy),&(g_IMU.gyroz));	//得到陀螺仪数据
//		}
		HAL_Delay(100);
//		FUZZY_Init(&fuzz_contrl,6.0f,0.0f,3.0f,300.0f,180.0f,-180.0f);
//	  fuzz_contrl.maxdKp=100.0f,
//	  fuzz_contrl.mindKp=-100.0f,
//	  fuzz_contrl.maxdKi=100.0f,
//	  fuzz_contrl.mindKi=-100.0f,
//	  fuzz_contrl.maxdKd=100.0f,
//	  fuzz_contrl.mindKd=-100.0f,
//	 Angle_begin=g_IMU.yaw;
//	 Angle_Set=Angle_begin;
	 //oled set
	 
	 Set_speed(0,0);
	
	// Eye_init();

}


void Main_Loop(void)
{


	
	 Walk_run();
	 

	 
}

void Main_Interrupt(void)
{
	 PID_EXECUTOR();
   Walk_run_interrupt();
}

