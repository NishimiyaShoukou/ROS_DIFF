/******************************************************************************
///
/// pid��������2021��3��
/// �ο�RM���Ʋ���
*******************************************************************************/
#ifndef __pid_H
#define __pid_H


enum {
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,

    POSITION_PID,
    DELTA_PID,
};


typedef struct    //����pid�ṹ��
{
    float kp;
    float ki;
    float kd;
	  float max_out;
	  float err[3];
	  float i_band;
	  int out;
	
	  float max_input;
	  float min_input;

} pid_t;

int Abs(int val);
void PID_Reset(pid_t *pid,float kp_get,float ki_get,float kd_get);
float PID_speed_contrl(pid_t *pid,float set,float get);  //pid��λʽ
int Incremental_PID(pid_t *pid,float set,float get);
float PID_contrl(pid_t *pid,float set,float get);
float PID_Calc(pid_t *pid,float set,float get);
extern pid_t pid_spd[2];   //��������������ĸ������ٶ�

extern  pid_t pid_turn;    //ת��pid
extern  pid_t pid_move;
#endif

