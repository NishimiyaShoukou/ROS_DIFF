#ifndef __chasis_H__
#define __chasis_H__


typedef enum
{
	
	START			,	//���ģʽ
	WAIT			, //�ȴ�ģʽ
	SWAP      , //ץ��ģʽ
	PLACE     , //����ģʽ
	TURN      , //ת��ģʽ
	RETURN      , //����ģʽ
}CarStateTypedef;


void Main_Setup(void);
void Main_Loop(void);
void Main_Interrupt(void);

extern CarStateTypedef Miracle;

#endif
