#include "pid.h"

#include <math.h>



pid_t pid_spd[2];
pid_t pid_move;
pid_t pid_turn;
/*****************
   ȡ����ֵ
	 ****************/
	 
int Abs(int val)
{
	return val>0?val:-val;
}


/***********************
    pid�����趨
************************/
void PID_Reset(pid_t *pid,float kp_get,float ki_get,float kd_get)
{
	  pid->kp=kp_get;
	  pid->ki=ki_get;
	  pid->kd=kd_get;
	
}
/*****************************************************
ע��PID������
 

****************************************************/

float PID_contrl(pid_t *pid,float set,float get)  //pid��λʽ
{
    
    
    pid->err[NOW]=set-get;                       //�õ���ǰƫ��ֵ
	  
	  if( pid->err[NOW] > 180)
    {
         pid->err[NOW]= pid->err[NOW]-360;
    }
    else if( pid->err[NOW] < -180)
    {
         pid->err[NOW]=360+ pid->err[NOW];
    }
	  pid->max_out=200;                               
	  pid->i_band+=pid->err[NOW];             
	  pid->i_band=(pid->i_band>200)?0:pid->i_band;    //�����޷�
	  pid->i_band=(pid->i_band<-200)?0:pid->i_band;
	  pid->out=pid->kp*pid->err[NOW]     +
	           pid->ki*pid->i_band       +
	           pid->kd*(pid->err[NOW]-2*pid->err[LAST]+pid->err[LLAST]);
	
	
	  pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
	  pid->out=(pid->out>pid->max_out)?pid->max_out:pid->out;      //����޷�
	  pid->out=(pid->out<-(pid->max_out))?-(pid->max_out):pid->out;
    return pid->out;
}
/*****************************************************
ע��PIDλ�ƿ�����
 

****************************************************/

float PID_Calc(pid_t *pid,float set,float get)  //pid��λʽ
{
    
    
    pid->err[NOW]=set-get;                       //�õ���ǰƫ��ֵ
	  
	 
	  pid->max_out=1000;                           
	  pid->i_band+=pid->err[NOW];             
	  pid->i_band=(pid->i_band>10000)?0:pid->i_band;    //�����޷�
	  pid->i_band=(pid->i_band<-10000)?0:pid->i_band;
	  pid->out=pid->kp*pid->err[NOW]     +
	           pid->ki*pid->i_band       +
	           pid->kd*(pid->err[NOW]-2*pid->err[LAST]+pid->err[LLAST]);
	 
	
	  pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
	  pid->out=(pid->out>pid->max_out)?pid->max_out:pid->out;      //����޷�
	  pid->out=(pid->out<-(pid->max_out))?-(pid->max_out):pid->out;
    return pid->out;
}

/*****************************************************
ע��PID�ٶȿ�����
 

****************************************************/

float PID_speed_contrl(pid_t *pid,float set,float get)  //pid��λʽ
{
    
    
    pid->err[NOW]=set-get;                       //�õ���ǰƫ��ֵ
	  
	 
	  pid->max_out=1000;                           
	  pid->i_band+=pid->err[NOW];             
	  pid->i_band=(pid->i_band>10000)?0:pid->i_band;    //�����޷�
	  pid->i_band=(pid->i_band<-10000)?0:pid->i_band;
	  pid->out=pid->kp*pid->err[NOW]     +
	           pid->ki*pid->i_band       +
	           pid->kd*(pid->err[NOW]-2*pid->err[LAST]+pid->err[LLAST]);
	 
	
	  pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
	  pid->out=(pid->out>pid->max_out)?pid->max_out:pid->out;      //����޷�
	  pid->out=(pid->out<-(pid->max_out))?-(pid->max_out):pid->out;
    return pid->out;
}
/**************************************************************************
�������ܣ�����PI������
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PID(pid_t *pid,float set,float get)
{   
     
       
     pid->err[NOW]=set-get;               //����ٶ�ƫ��ɲ���ֵ��ȥĿ��ֵ��
     pid->out+=pid->kp*(pid->err[NOW]-pid->err[LAST])+
	             pid->ki*pid->err[NOW]                 +  //ʹ������ PI ������������ PWM��
	             pid->kd*(pid->err[NOW]-2*pid->err[LAST]+pid->err[LLAST]);
     pid->err[LLAST] = pid->err[LAST];
     pid->err[LAST] = pid->err[NOW];     //������һ��ƫ�� 
	 pid->max_out=1000;  
	 pid->out=(pid->out>pid->max_out)?pid->max_out:pid->out;      //����޷�
	 pid->out=(pid->out<-(pid->max_out))?-(pid->max_out):pid->out;
     return pid->out;                         //�������
}