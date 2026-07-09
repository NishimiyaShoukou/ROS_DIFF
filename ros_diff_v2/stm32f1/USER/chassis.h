/******************************************************************************
 * @file    chassis.h
 * @brief   Chassis control interfaces and motion state declarations.
 * @author  User
 * @date    2026-05-19
 * @version V1.0.0
 * @note    Part of the STM32F1 robot control project.
 ******************************************************************************/

#ifndef __chassis_H__
#define __chassis_H__


typedef enum
{
	
	START			,	//冲刺模式
	WAIT			, //等待模式
	SWAP      , //抓球模式
	PLACE     , //放球模式
	TURN      , //转向模式
	RETURN      , //返回模式
}CarStateTypedef;


void main_setup(void);
void main_loop(void);
void Main_Interrupt(void);

extern CarStateTypedef Miracle;
#define CHASSIS_SPEED_LOOP_PERIOD_MS 10U
#define ROBOT_COMM_PERIOD_MS          20U
#define UART2_TTL_TEST_ENABLE      0U
#define UART2_TTL_TEST_PERIOD_MS   1000U
#endif
