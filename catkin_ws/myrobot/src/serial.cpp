#include "serial.h"
 
using namespace std;
using namespace boost::asio;
//������ض���
boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev, "/dev/chasis");
boost::system::error_code err;
/********************************************************
            ���ڷ��ͽ�����س��������������������
********************************************************/
const unsigned char header[2]  = {0x55, 0xaa};
const unsigned char ender[2]   = {0x0d, 0x0a};

union sendData
{
	short speed;
	unsigned char data_speed[2];
}leftVelSet,rightVelSet;
 
//�������ݣ������١������١��Ƕȣ������壨-32767 - +32768��
union receiveData
{
	short d;
	unsigned char data[2];
}leftVelNow,rightVelNow,angleNow;
 
/********************************************************
�������ܣ����ڲ�����ʼ��
��ڲ�������
���ڲ�����
********************************************************/
void serialInit()
{
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));    
}
 
/********************************************************
�������ܣ����Ի����˵��������ӿ����ٶȣ�������͸���λ��
��ڲ��������������ٶȡ����ٶ�
���ڲ�����
********************************************************/
void writeSpeed(double Left_v, double Right_v,unsigned char ctrlFlag)
{
    unsigned char buf[13] = {0};//
    int i, length = 0;
 
    leftVelSet.speed = Left_v;//mm/s
    rightVelSet.speed= Right_v;
 
    // ������Ϣͷ
    for(i = 0; i < 2; i++)
        buf[i] = header[i];             //buf[0]  buf[1]
    
    // ���û������������ٶ�
    length = 5;
    buf[2] = length;                    //buf[2]
    for(i = 0; i < 2; i++)
    {
        buf[i + 3] = leftVelSet.data_speed[i];  //buf[3] buf[4]
        buf[i + 5] = rightVelSet.data_speed[i]; //buf[5] buf[6]
    }
    // Ԥ������ָ��
    buf[3 + length - 1] = ctrlFlag;       //buf[7]
 
    
    buf[3 + length] = ender[0];     //buf[8]
    buf[3 + length + 1] = ender[1];     //buf[9]
 
    // ͨ�������·�����
    boost::asio::write(sp, boost::asio::buffer(buf));
}
/********************************************************
�������ܣ�����λ����ȡ����
��ڲ������������������١��������١��Ƕȣ�Ԥ������λ
���ڲ�����bool
********************************************************/
bool readSpeed(double &Left_v,double &Right_v,double &Angle,unsigned char &ctrlFlag)
{
    char i, length = 0;
    unsigned char checkSum;
    unsigned char buf[1024]={0};
    //=========================================================
    //�˶δ�����Զ����ݵĽ�β�����������ж�ȡ���ݵ�ͷ��
    try
    {
        boost::asio::streambuf response;
        // boost::asio::read(sp , boost::asio::buffer(buf,12 ) );
        
        boost::asio::read_until(sp, response, "\r\n",err);   
        copy(istream_iterator<unsigned char>(istream(&response)>>noskipws),
        istream_iterator<unsigned char>(),
        buf); 
    }  
    catch(boost::system::system_error &err)
    {
        ROS_INFO("read_until error");
    } 
    //=========================================================        
 
    // �����Ϣͷ
    if (buf[0]!= header[0] || buf[1] != header[1])   //buf[0] buf[1]
    {
        ROS_ERROR("Received message header error!");
        return false;
    }
    // ���ݳ���
    length = buf[2];                                 //buf[2]
 
    // �����ϢУ��ֵ
    // checkSum = getCrc8(buf, 3 + length);             //buf[10] ����ó�
    // if (checkSum != buf[3 + length])                 //buf[10] ���ڽ���
    // {
    //     ROS_ERROR("Received data check sum error!");
    //     return false;
    // }    
 
    // ��ȡ�ٶ�ֵ
    for(i = 0; i < 2; i++)
    {
        leftVelNow.data[i]  = buf[i + 3]; //buf[3] buf[4]
        rightVelNow.data[i] = buf[i + 5]; //buf[5] buf[6]
        angleNow.data[i]    = buf[i + 7]; //buf[7] buf[8]
    }
 
    // ��ȡ���Ʊ�־λ
    ctrlFlag = buf[9];
    
    Left_v  =-leftVelNow.d;
    Right_v =-rightVelNow.d;
    Angle   =angleNow.d;
 
    return true;
}
/********************************************************
�������ܣ����8λѭ������У��ֵ
��ڲ����������ַ������
���ڲ�����У��ֵ
********************************************************/
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
    unsigned char crc;
    unsigned char i;
    crc = 0;
    while(len--)
    {
        crc ^= *ptr++;
        for(i = 0; i < 8; i++)
        {
            if(crc&0x01)
                crc=(crc>>1)^0x8C;
            else 
                crc >>= 1;
        }
    }
    return crc;
}