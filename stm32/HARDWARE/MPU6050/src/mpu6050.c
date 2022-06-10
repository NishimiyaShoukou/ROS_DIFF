#include "mpu6050.h"
//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
	uint8_t i; 
  I2C_Start(); 
	I2C_SendByte((addr<<1)|0);//����������ַ+д����	
	if(I2C_WaitAck() == 0)	//�ȴ�Ӧ��
	{
		I2C_Stop();		 
		return 1;		
	}
    I2C_SendByte(reg);	//д�Ĵ�����ַ
    I2C_WaitAck();		//�ȴ�Ӧ��
	for(i=0;i<len;i++)
	{
		I2C_SendByte(buf[i]);	//��������
		if(I2C_WaitAck() == 0)		//�ȴ�ACK
		{
			I2C_Stop();	 
			return 1;		 
		}		
	}    
  I2C_Stop();	 
	return 0;	
} 
//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
 	I2C_Start(); 
	I2C_SendByte((addr<<1)|0);//����������ַ+д����	
	if(I2C_WaitAck() == 0)	//�ȴ�Ӧ��
	{
		I2C_Stop();		 
		return 1;		
	}
    I2C_SendByte(reg);	//д�Ĵ�����ַ
    I2C_WaitAck();		//�ȴ�Ӧ��
    I2C_Start();
	I2C_SendByte((addr<<1)|1);//����������ַ+������	
    I2C_WaitAck();		//�ȴ�Ӧ�� 
	while(len)
	{
		if(len==1)
		{
			*buf=I2C_RadeByte();//������,����nACK
			I2C_NoAck();
		}			
		else
		{
			*buf=I2C_RadeByte();	//������,����ACK 
			I2C_Ack();
		}			
		len--;
		buf++; 
	}    
    I2C_Stop();	//����һ��ֹͣ���� 
	return 0;	
}

//IICдһ���ֽ�
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Single_Write(uint8_t reg,uint8_t data) 				 
{
  I2C_Start();
	I2C_SendByte((MPU_ADDR<<1)|0);//����������ַ+д����	
	if(I2C_WaitAck() == 0)	//�ȴ�Ӧ��
	{
		I2C_Stop();		 
		return 1;		
	}
  I2C_SendByte(reg);	//д�Ĵ�����ַ
  I2C_WaitAck();		//�ȴ�Ӧ�� 
	I2C_SendByte(data);//��������
	if(I2C_WaitAck() == 0)	//�ȴ�ACK
	{
		I2C_Stop();	 
		return 1;		 
	}		 
    I2C_Stop();	 
	return 0;
}

//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
uint8_t MPU_Read_Byte(uint8_t reg)
{
	uint8_t res;
  I2C_Start(); 
	I2C_SendByte((MPU_ADDR<<1)|0);//����������ַ+д����	
	I2C_WaitAck();		//�ȴ�Ӧ�� 
	I2C_SendByte(reg);	//д�Ĵ�����ַ
	I2C_WaitAck();		//�ȴ�Ӧ��
	I2C_Start();
	I2C_SendByte((MPU_ADDR<<1)|1);//����������ַ+������	
  I2C_WaitAck();		//�ȴ�Ӧ�� 
	res=I2C_RadeByte();//��ȡ����,����nACK 
	I2C_NoAck();
  I2C_Stop();			//����һ��ֹͣ���� 
	return res;		
}

//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Single_Write(MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ  
}
//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Single_Write(MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}
//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Single_Write(MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
}
//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Single_Write(MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MPU_Get_Temperature(void)
{
    uint8_t buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((uint16_t)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    uint8_t buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((uint16_t)buf[0]<<8)|buf[1];  
		*gy=((uint16_t)buf[2]<<8)|buf[3];  
		*gz=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    uint8_t buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((uint16_t)buf[0]<<8)|buf[1];  
		*ay=((uint16_t)buf[2]<<8)|buf[3];  
		*az=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//��ʼ��MPU6050
//����ֵ:0,�ɹ�
//    ����,�������
uint8_t MPU_Init(void)
{
	uint8_t res;
	
	I2C_GPIO_Config();//��ʼ��IIC����
	MPU_Single_Write(MPU_PWR_MGMT1_REG,0X80);	//��λMPU6050
	HAL_Delay(100);
	MPU_Single_Write(MPU_PWR_MGMT1_REG,0X00);	//����MPU6050 
	MPU_Set_Gyro_Fsr(3);					//�����Ǵ�����,��2000dps
	MPU_Set_Accel_Fsr(0);					//���ٶȴ�����,��2g
	MPU_Set_Rate(50);						//���ò�����50Hz
	MPU_Single_Write(MPU_INT_EN_REG,0X00);	//�ر������ж�
	MPU_Single_Write(MPU_USER_CTRL_REG,0X00);	//I2C��ģʽ�ر�
	MPU_Single_Write(MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
	MPU_Single_Write(MPU_INTBP_CFG_REG,0X80);	//INT���ŵ͵�ƽ��Ч
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//����ID��ȷ
	{
		MPU_Single_Write(MPU_PWR_MGMT1_REG,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
		MPU_Single_Write(MPU_PWR_MGMT2_REG,0X00);	//���ٶ��������Ƕ�����
		MPU_Set_Rate(50);						//���ò�����Ϊ50Hz
	}else return 1;
	return 0;
}

