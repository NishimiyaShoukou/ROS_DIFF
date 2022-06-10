#include "fuzzypid.h"

//int fuzzytab[7]=[-6,-4,-2,0,2,4,6];

fuzzy_pid fuzz_contrl;
//kpģ������
const int ruleKp[7][7]={
	                NB,NB,NM,NM,NS,ZO,ZO,
									NB,NB,NM,NS,NS,ZO,ZO,
									NB,NM,NS,NS,ZO,PS,ZO,
									NM,NM,NS,ZO,PS,PM,PM,
									NM,NS,ZO,PS,PS,PM,PB,
									ZO,ZO,PS,PS,PM,PB,PB,
									ZO,ZO,PS,PM,PM,PB,PB,};
	
//kiģ������
const int ruleKi[7][7]={NB,NB,NM,NM,NS,ZO,ZO,
									NB,NB,NM,NS,NS,ZO,ZO,
									NB,NM,NS,NS,ZO,PS,PS,
									NM,NM,NS,ZO,PS,PM,PM,
									NM,NS,ZO,PS,PS,PM,PB,
									ZO,ZO,PS,PS,PM,PB,PB,
									ZO,ZO,PS,PM,PM,PB,PB,};


//kdģ������
const int ruleKd[7][7]={PS,NS,NB,NB,NB,NM,PS,
									PS,NS,NB,NM,NM,NS,ZO,
									ZO,NS,NM,NM,NS,NS,ZO,
									ZO,NS,NS,NS,NS,NS,ZO,
									ZO,ZO,ZO,ZO,ZO,ZO,ZO,
									PB,NS,PS,PS,PS,PS,PB,
									PB,PM,PM,PM,PS,PS,PB,};
      

/*****************
   ȡ����ֵ
	 ****************/
//	 
//int Abs(int val)
//{
//	return val>0?val:-val;
//}

/***********************
    pid��ʼ��
************************/
void FUZZY_Init(fuzzy_pid *vPID,float kp_get,float ki_get,float kd_get,float maxout,float maxinput,float mininput)
{
	  vPID->fuzzy_kp=kp_get;
	  vPID->fuzzy_ki=ki_get;
	  vPID->fuzzy_kd=kd_get;
	  vPID->fuzzy_max_out=maxout;
	
	  vPID->max_input=maxinput;
	  vPID->min_input=mininput;
	
}
///***********************
//    pid�����趨
//************************/
//void PID_Reset(fuzzy_pid *pid,float kp_get,float ki_get,float kd_get)
//{
//	  pid->fuzzy_kp=kp_get;
//	  pid->fuzzy_ki=ki_get;
//	  pid->fuzzy_kd=kd_get;
//	
//}
/*����������������,����{-6��-5��-4��-3��-2��-1��0��1��2��3��4��5��6}*/
//pv��ǰֵ
static void LinearQuantization(fuzzy_pid *vPID,float pv,float *qValue)
{
  float thisError;
  float deltaError;
 
  thisError=vPID->set-pv;                  //����ƫ��ֵ
  deltaError=thisError-vPID->fuzzy_err[FUZZY_LAST];         //����ƫ������
 
  qValue[0]=6.0*thisError/(vPID->max_input-vPID->min_input);
  qValue[1]=3.0*deltaError/(vPID->max_input-vPID->min_input);
}
//�޷�����
float LinearRealization(int max,int min,float value)
{
	value=value>=max?max:value;
	value=value<=min?min:value;
	return value;
}
/*�����ȼ��㺯��*/
/*
  Ŀǰ�����Ȼ���Ϊ NB,NM,NS,ZO,PS,PM,PB.

*/
static void CalcMembership(float *ms,float qv,int * index)
{
  if((qv>=-NB)&&(qv<-NM))
  {
    index[0]=0;
    index[1]=1;
    ms[0]=-0.5*qv-2.0;  //y=-0.5x-2.0
    ms[1]=0.5*qv+3.0;   //y=0.5x+3.0
  }
  else if((qv>=-NM)&&(qv<-NS))
  {
    index[0]=1;
    index[1]=2;
    ms[0]=-0.5*qv-1.0;  //y=-0.5x-1.0
    ms[1]=0.5*qv+2.0;   //y=0.5x+2.0
  }
  else if((qv>=-NS)&&(qv<ZO))
  {
    index[0]=2;
    index[1]=3;
    ms[0]=-0.5*qv;      //y=-0.5x
    ms[1]=0.5*qv+1.0;   //y=0.5x+1.0
  }
  else if((qv>=ZO)&&(qv<PS))
  {
    index[0]=3;
    index[1]=4;
    ms[0]=-0.5*qv+1.0;  //y=-0.5x+1.0
    ms[1]=0.5*qv;       //y=0.5x
  }
  else if((qv>=PS)&&(qv<PM))
  {
    index[0]=4;
    index[1]=5;
    ms[0]=-0.5*qv+2.0;  //y=-0.5x+2.0
    ms[1]=0.5*qv-1.0;   //y=0.5x-1.0
  }
  else if((qv>=PM)&&(qv<=PB))
  {
    index[0]=5;
    index[1]=6;
    ms[0]=-0.5*qv+3.0;  //y=-0.5x+3.0
    ms[1]=0.5*qv-2.0;   //y=0.5x-2.0
  }
}


/*��ģ��������,���ݾ�������������������Ⱥ�������*/
static void Fuzzy_Calc(fuzzy_pid *vPID,float pv,float *deltaK)
{
  float qValue[2]={0,0};        //ƫ�������������ֵ
  int indexE[2]={0,0};          //ƫ������������
  float msE[2]={0,0};           //ƫ��������
  int indexEC[2]={0,0};         //ƫ����������������
  float msEC[2]={0,0};          //ƫ������������
  float qValueK[3];
 
  LinearQuantization(vPID,pv,qValue);
 
  CalcMembership(msE,qValue[0],indexE);
  CalcMembership(msEC,qValue[1],indexEC);
 
  qValueK[0]=msE[0]*(msEC[0]*ruleKp[indexE[0]][indexEC[0]]+msEC[1]*ruleKp[indexE[0]][indexEC[1]])
            +msE[1]*(msEC[0]*ruleKp[indexE[1]][indexEC[0]]+msEC[1]*ruleKp[indexE[1]][indexEC[1]]);
  qValueK[1]=msE[0]*(msEC[0]*ruleKi[indexE[0]][indexEC[0]]+msEC[1]*ruleKi[indexE[0]][indexEC[1]])
            +msE[1]*(msEC[0]*ruleKi[indexE[1]][indexEC[0]]+msEC[1]*ruleKi[indexE[1]][indexEC[1]]);
  qValueK[2]=msE[0]*(msEC[0]*ruleKd[indexE[0]][indexEC[0]]+msEC[1]*ruleKd[indexE[0]][indexEC[1]])
            +msE[1]*(msEC[0]*ruleKd[indexE[1]][indexEC[0]]+msEC[1]*ruleKd[indexE[1]][indexEC[1]]);
 
  deltaK[0]=LinearRealization(vPID->maxdKp,vPID->mindKp,qValueK[0]);
  deltaK[1]=LinearRealization(vPID->maxdKi,vPID->mindKi,qValueK[1]);
  deltaK[2]=LinearRealization(vPID->maxdKd,vPID->mindKd,qValueK[2]);
}


/*****************************************************
ע��PID������
 

****************************************************/

float Fuzzy_pid(fuzzy_pid *vPID,float set,float get)  //pid��λʽ
{
	  float kpscal=0.9,kiscal=0.5,kdscal=0.15;
	  float Final_kp,Final_ki,Final_kd; 
    static float delta[3];
    
    vPID->fuzzy_err[FUZZY_NOW]=set-get;                       //�õ���ǰƫ��ֵ
	  vPID->set=set;
	 
	  Fuzzy_Calc(vPID,get,delta); 
    Final_kp = vPID->fuzzy_kp+kpscal*delta[0];
	  Final_ki = vPID->fuzzy_ki+kiscal*delta[1];
	  Final_kd = vPID->fuzzy_kd+kdscal*delta[2];
	  
	  vPID->fuzzy_i_band+=vPID->fuzzy_err[FUZZY_NOW];             
	  vPID->fuzzy_i_band=(vPID->fuzzy_i_band>10000)?0:vPID->fuzzy_i_band;    //�����޷�
	  vPID->fuzzy_i_band=(vPID->fuzzy_i_band<-10000)?0:vPID->fuzzy_i_band;
	  vPID->fuzzy_out=Final_kp*vPID->fuzzy_err[FUZZY_NOW]     +
	           Final_ki*vPID->fuzzy_i_band       +
	           Final_kd*(vPID->fuzzy_err[FUZZY_NOW]-2*vPID->fuzzy_err[FUZZY_LAST]+vPID->fuzzy_err[FUZZY_LLAST]);
	
	
	  vPID->fuzzy_err[FUZZY_LLAST] = vPID->fuzzy_err[FUZZY_LAST];
    vPID->fuzzy_err[FUZZY_LAST] = vPID->fuzzy_err[FUZZY_NOW];
	  vPID->fuzzy_out=(vPID->fuzzy_out>vPID->fuzzy_max_out)?vPID->fuzzy_max_out:vPID->fuzzy_out;      //����޷�
	  vPID->fuzzy_out=(vPID->fuzzy_out<-(vPID->fuzzy_max_out))?-(vPID->fuzzy_max_out):vPID->fuzzy_out;
    return vPID->fuzzy_out;
}

