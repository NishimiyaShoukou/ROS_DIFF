/******************************************************************************
 * @file    pid.h
 * @brief   PID controller data structures and public interfaces.
 * @author  User
 * @date    2026-05-19
 * @version V1.0.0
 * @note    Initial PID module referenced RM control design, 2021-03.
 ******************************************************************************/

#ifndef __pid_H
#define __pid_H


enum {
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,

    POSITION_PID,
    DELTA_PID,
};


typedef struct    //定义pid结构体
{
    float kp;
    float ki;
    float kd;
	  float max_out;
		float max_step;
	  float err[3];
	  float i_band;
		float i_max;
		float last_out;
	  int out;
	
	  float max_input;
	  float min_input;

} pid_t;

int abs(int val);
void pid_reset(pid_t *pid,float kp_get,float ki_get,float kd_get);
float pid_speed_contrl(pid_t *pid,float set,float get);  //pid，位式
int incremental_pid(pid_t *pid,float set,float get);
float pid_contrl(pid_t *pid,float set,float get);
float pid_calc(pid_t *pid,float set,float get);
extern pid_t pid_spd[2];   //经过计算后分配给四个轮子速度

extern  pid_t pid_turn;    //转向pid
extern  pid_t pid_move;
#endif

