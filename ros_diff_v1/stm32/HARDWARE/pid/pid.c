#include "pid.h"

#include <math.h>



pid_t pid_spd[2];
pid_t pid_move;
pid_t pid_turn;
/*****************
   取绝对值
	 ****************/
	 
int Abs(int val)
{
	return val>0?val:-val;
}


/***********************
    pid参数设定
************************/
void PID_Reset(pid_t *pid,float kp_get,float ki_get,float kd_get)
{
	  pid->kp=kp_get;
	  pid->ki=ki_get;
	  pid->kd=kd_get;
	
}
/*****************************************************
注：PID控制器
 

****************************************************/

float PID_contrl(pid_t *pid,float set,float get)  //pid，位式
{
    
    
    pid->err[NOW]=set-get;                       //得到当前偏差值
	  
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
	  pid->i_band=(pid->i_band>200)?0:pid->i_band;    //积分限幅
	  pid->i_band=(pid->i_band<-200)?0:pid->i_band;
	  pid->out=pid->kp*pid->err[NOW]     +
	           pid->ki*pid->i_band       +
	           pid->kd*(pid->err[NOW]-2*pid->err[LAST]+pid->err[LLAST]);
	
	
	  pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
	  pid->out=(pid->out>pid->max_out)?pid->max_out:pid->out;      //输出限幅
	  pid->out=(pid->out<-(pid->max_out))?-(pid->max_out):pid->out;
    return pid->out;
}
/*****************************************************
注：PID位移控制器
 

****************************************************/

float PID_Calc(pid_t *pid,float set,float get)  //pid，位式
{
    
    
    pid->err[NOW]=set-get;                       //得到当前偏差值
	  
	 
	  pid->max_out=1000;                           
	  pid->i_band+=pid->err[NOW];             
	  pid->i_band=(pid->i_band>10000)?0:pid->i_band;    //积分限幅
	  pid->i_band=(pid->i_band<-10000)?0:pid->i_band;
	  pid->out=pid->kp*pid->err[NOW]     +
	           pid->ki*pid->i_band       +
	           pid->kd*(pid->err[NOW]-2*pid->err[LAST]+pid->err[LLAST]);
	 
	
	  pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
	  pid->out=(pid->out>pid->max_out)?pid->max_out:pid->out;      //输出限幅
	  pid->out=(pid->out<-(pid->max_out))?-(pid->max_out):pid->out;
    return pid->out;
}

/*****************************************************
注：PID速度控制器
 

****************************************************/

float PID_speed_contrl(pid_t *pid,float set,float get)  //pid，位式
{
    
    
    pid->err[NOW]=set-get;                       //得到当前偏差值
	  
	 
	  pid->max_out=1000;                           
	  pid->i_band+=pid->err[NOW];             
	  pid->i_band=(pid->i_band>10000)?0:pid->i_band;    //积分限幅
	  pid->i_band=(pid->i_band<-10000)?0:pid->i_band;
	  pid->out=pid->kp*pid->err[NOW]     +
	           pid->ki*pid->i_band       +
	           pid->kd*(pid->err[NOW]-2*pid->err[LAST]+pid->err[LLAST]);
	 
	
	  pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
	  pid->out=(pid->out>pid->max_out)?pid->max_out:pid->out;      //输出限幅
	  pid->out=(pid->out<-(pid->max_out))?-(pid->max_out):pid->out;
    return pid->out;
}
/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PID(pid_t *pid,float set,float get)
{   
     
       
     pid->err[NOW]=set-get;               //求出速度偏差，由测量值减去目标值。
     pid->out+=pid->kp*(pid->err[NOW]-pid->err[LAST])+
	             pid->ki*pid->err[NOW]                 +  //使用增量 PI 控制器求出电机 PWM。
	             pid->kd*(pid->err[NOW]-2*pid->err[LAST]+pid->err[LLAST]);
     pid->err[LLAST] = pid->err[LAST];
     pid->err[LAST] = pid->err[NOW];     //保存上一次偏差 
	 pid->max_out=1000;  
	 pid->out=(pid->out>pid->max_out)?pid->max_out:pid->out;      //输出限幅
	 pid->out=(pid->out<-(pid->max_out))?-(pid->max_out):pid->out;
     return pid->out;                         //增量输出
}