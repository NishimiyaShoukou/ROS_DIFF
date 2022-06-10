#include "move.h"
#include "stm32f4xx.h"
#include "tim.h"
#include "contrl.h"
#include "comHandle.h"
#include "usart.h"
#include "pid.h"
#include "chasis.h"
#include "track.h"
#include "oled.h"
#include "stdio.h"
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
//#include "pstwo.h"

Map_machine Chasis_up;



/**********************************************************************************************************
*�� �� ��: goto_1m
*����˵��: ֱ����һ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
static int set_des_flag=0;
void goto_1m(void){
	if(set_des_flag == 0){
		Chasis_up.SetPoint = 100;//1m
		set_des_flag = 1;
	}	
	Chasis_Loop.Motor_set_speed[1] = LIMIT_MAX_MIN(PID_Calc(&pid_move, -Chasis_up.SetPoint,Chasis_Loop.positionNow),35,-35);
	Chasis_Loop.Motor_set_speed[0]=Chasis_Loop.Motor_set_speed[1];
	
}


void Walk_run(void)
{
//	Set_speed(20,0);
	//Set_Pwm(-600,-600);
	//USART1->DR='D';
	//printf("yaw: %f\n\r",g_IMU.yaw);
	//Usartdata_up(Chasis_Loop.Motor_get_speed[0],Chasis_Loop.Motor_get_speed[1],(short)g_IMU.yaw,'a');
	
	ImuDataUpdate();      //��������������
}
void Walk_run_interrupt(void)
{
	  
	  
	 
}
