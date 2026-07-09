/******************************************************************************
 * @file    pid.c
 * @brief   PID controller calculation and parameter management routines.
 * @author  User
 * @date    2026-05-19
 * @version V1.0.0
 * @note    Part of the STM32F1 robot control project.
 ******************************************************************************/

#include "pid.h"

#include <math.h>

/* Integral separation threshold.
 * When |err| is larger than this, the I-term is skipped for that cycle.
 * Prevents integral wind-up on direction reversals / large step changes. */
#define INCREMENTAL_PID_I_SEPARATION  40.0f



pid_t pid_spd[2];
pid_t pid_move;
pid_t pid_turn;
/*****************
   取出绝对值
	 ****************/
	 
int abs(int val)
{
	return val>0?val:-val;
}


/***********************
    pid 参数设定
************************/
void pid_reset(pid_t *pid,float kp_get,float ki_get,float kd_get)
{
	  pid->kp=kp_get;
	  pid->ki=ki_get;
	  pid->kd=kd_get;
	
}
/*****************************************************
增量式 PID 控制器
 

****************************************************/

float pid_contrl(pid_t *pid,float set,float get)	//pid 位置式
{
    
    
        pid->err[NOW]=set-get;	//得到当前偏差值
	  
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
	  	  pid->i_band=(pid->i_band>200)?200:pid->i_band;	//积分限幅
	  pid->i_band=(pid->i_band<-200)?-200:pid->i_band;
	  pid->out=pid->kp*pid->err[NOW]     +
	           pid->ki*pid->i_band       +
	           pid->kd*(pid->err[NOW]-2*pid->err[LAST]+pid->err[LLAST]);
	
	
	  pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
	  	  pid->out=(pid->out>pid->max_out)?pid->max_out:pid->out;	//输出限幅
	  pid->out=(pid->out<-(pid->max_out))?-(pid->max_out):pid->out;
    return pid->out;
}
/*****************************************************
位置式 PID 控制器
 

****************************************************/

float pid_calc(pid_t *pid,float set,float get)	//pid 位置式
{
    
    
        pid->err[NOW]=set-get;	//得到当前偏差值
	  
	 
	  pid->max_out=1000;                           
	  pid->i_band+=pid->err[NOW];             
	  	  pid->i_band=(pid->i_band>10000)?0:pid->i_band;	//积分限幅
	  pid->i_band=(pid->i_band<-10000)?0:pid->i_band;
	  pid->out=pid->kp*pid->err[NOW]     +
	           pid->ki*pid->i_band       +
	           pid->kd*(pid->err[NOW]-2*pid->err[LAST]+pid->err[LLAST]);
	 
	
	  pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
	  	  pid->out=(pid->out>pid->max_out)?pid->max_out:pid->out;	//输出限幅
	  pid->out=(pid->out<-(pid->max_out))?-(pid->max_out):pid->out;
    return pid->out;
}

/*****************************************************
速度式 PID 控制器
 

****************************************************/

float pid_speed_contrl(pid_t *pid,float set,float get)	//pid 位置式
{
    
    
        pid->err[NOW]=set-get;	//得到当前偏差值
	  
	 
	  pid->max_out=1000;                           
	  pid->i_band+=pid->err[NOW];             
	  	  pid->i_band=(pid->i_band>10000)?10000:pid->i_band;	//积分限幅
	  pid->i_band=(pid->i_band<-10000)?-10000:pid->i_band;
	  pid->out=pid->kp*pid->err[NOW]     +
	           pid->ki*pid->i_band       +
	           pid->kd*(pid->err[NOW]-2*pid->err[LAST]+pid->err[LLAST]);
	 
	
	  pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
	  	  pid->out=(pid->out>pid->max_out)?pid->max_out:pid->out;	//输出限幅
	  pid->out=(pid->out<-(pid->max_out))?-(pid->max_out):pid->out;
    return pid->out;
}
/**************************************************************************
函数功能：增量 PI 控制器
入口参数：pid 结构体指针，设定值，实际值
返回  值：输出 PWM
离散 PID 增量式公式：
pwm += Kp * (e(k) - e(k-1)) + Ki * e(k) + Kd * (e(k) - 2*e(k-1) + e(k-2))
e(k)   本次偏差
e(k-1) 上一次的偏差  以此类推
pwm    输出控制量
对于速度控制闭环，只使用 PI 控制（即去掉 Kd 项）：
pwm += Kp * (e(k) - e(k-1)) + Ki * e(k)
**************************************************************************/
int incremental_pid(pid_t *pid, float set, float get) {
    // 1. 计算当前误差
    float err = set - get;

    // 2. 死区处理 (已注释)
//    if (fabsf(err) < pid->deadband) {
//        err = 0.0f;
//    }

    // 3. 保存当前误差，先保留 NOW 槽位以便后续读取
    pid->err[0] = err;   // NOW

    // 4. 计算原始增量，按标准增量式公式：
    /* Integral separation: skip I-term when |err| is large to prevent wind-up. */
    float i_term = (fabsf(pid->err[0]) > INCREMENTAL_PID_I_SEPARATION)
                       ? 0.0f
                       : (pid->ki * pid->err[0]);

    float delta_u =
                pid->kp * (pid->err[0] - pid->err[1]) +	// P 项
                i_term +	// I 项
                pid->kd * (pid->err[0] - 2*pid->err[1] + pid->err[2]);	// D 项

    // 5. 积分抗饱和：当输出接近饱和且当前积分项继续推动远离饱和时，临时取消积分
    float output_before = pid->out;
    if (output_before >= pid->max_out && i_term > 0.0f) {
                delta_u -= i_term;	// 取消积分位，防超调
    }
    if (output_before <= -pid->max_out && i_term < 0.0f) {
                delta_u -= i_term;	// 取消积分位，防超调
    }

    // 6. 增量限幅（防止阶跃突变）
    float new_out = output_before + delta_u;
    float step = new_out - pid->last_out;
    if (step > pid->max_step) step = pid->max_step;
    if (step < -pid->max_step) step = -pid->max_step;
    pid->out = pid->last_out + step;
    pid->last_out = pid->out;

    // 7. 输出限幅
    if (pid->out > pid->max_out) pid->out = pid->max_out;
    if (pid->out < -pid->max_out) pid->out = -pid->max_out;

    // 8. 更新历史误差（注意顺序：LLAST = LAST, LAST = NOW）
    pid->err[2] = pid->err[1];   // LLAST = LAST
    pid->err[1] = pid->err[0];   // LAST  = NOW

    // 9. 返回本次输出 (PWM 占空比)
    return (int)pid->out;
}
