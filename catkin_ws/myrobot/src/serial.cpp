/**
 * @file serial.cpp
 * @author thoelc
 * @brief 基于boost::asio方法的串口通信，优点是不用配置RosSerial包直接可以用
 * @version 0.1
 * @date 2023-11-06
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "serial.h"
 
using namespace std;
using namespace boost::asio;

boost::asio::io_service iosev;
// 绑定的串口号，根据自己情况更改
boost::asio::serial_port sp(iosev, "/dev/chasis");
boost::system::error_code err;
/********************************************************
 *  定义帧头和帧尾数据包
********************************************************/
const unsigned char header[2]  = {0x55, 0xaa};
const unsigned char ender[2]   = {0x0d, 0x0a};

union sendData
{
	short speed;
	unsigned char data_speed[2];
}leftVelSet,rightVelSet;
 
// 使用联合体定义数据包d和data共用内存（-32767 - +32768)
union receiveData
{
	short d;
	unsigned char data[2];
}leftVelNow,rightVelNow,angleNow;
 
/********************************************************
串口初始化波特率115200
********************************************************/
void serialInit()
{
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));    
}
 

/**
 * @brief 写速度函数，第三个参数保留添加功能
 * 
 * @param Left_v 
 * @param Right_v 
 * @param ctrlFlag 
 */
void writeSpeed(double Left_v, double Right_v,unsigned char ctrlFlag)
{
    unsigned char buf[13] = {0};//
    int i, length = 0;
 
    leftVelSet.speed = Left_v;//mm/s
    rightVelSet.speed= Right_v;
 
    // 写入帧头
    for(i = 0; i < 2; i++)
        buf[i] = header[i];             //buf[0]  buf[1]
    
    // 锟斤拷锟矫伙拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟劫讹拷
    length = 5;
    buf[2] = length;                    //buf[2]
    for(i = 0; i < 2; i++)
    {
        buf[i + 3] = leftVelSet.data_speed[i];  //buf[3] buf[4]
        buf[i + 5] = rightVelSet.data_speed[i]; //buf[5] buf[6]
    }
    // 预锟斤拷锟斤拷锟斤拷指锟斤拷
    buf[3 + length - 1] = ctrlFlag;       //buf[7]
 
    
    buf[3 + length] = ender[0];     //buf[8]
    buf[3 + length + 1] = ender[1];     //buf[9]
 
    // 通锟斤拷锟斤拷锟斤拷锟铰凤拷锟斤拷锟斤拷
    boost::asio::write(sp, boost::asio::buffer(buf));
}

/**
 * @brief 读速度函数
 * 
 * @param Left_v 
 * @param Right_v 
 * @param Angle 
 * @param ctrlFlag 
 * @return true 
 * @return false 
 */
bool readSpeed(double &Left_v,double &Right_v,double &Angle,unsigned char &ctrlFlag)
{
    char i, length = 0;
    unsigned char checkSum;
    unsigned char buf[1024]={0};
    //=========================================================
    //锟剿段达拷锟斤拷锟斤拷远锟斤拷锟斤拷莸慕锟轿诧拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷卸锟饺★拷锟斤拷莸锟酵凤拷锟?
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
 
    // 锟斤拷锟斤拷锟较⑼?
    if (buf[0]!= header[0] || buf[1] != header[1])   //buf[0] buf[1]
    {
        ROS_ERROR("Received message header error!");
        return false;
    }
    // 锟斤拷锟捷筹拷锟斤拷
    length = buf[2];                                 //buf[2]
 
    // 锟斤拷锟斤拷锟较⑿ｏ拷锟街?
    // checkSum = getCrc8(buf, 3 + length);             //buf[10] 锟斤拷锟斤拷贸锟?
    // if (checkSum != buf[3 + length])                 //buf[10] 锟斤拷锟节斤拷锟斤拷
    // {
    //     ROS_ERROR("Received data check sum error!");
    //     return false;
    // }    
 
    // 锟斤拷取锟劫讹拷值
    for(i = 0; i < 2; i++)
    {
        leftVelNow.data[i]  = buf[i + 3]; //buf[3] buf[4]
        rightVelNow.data[i] = buf[i + 5]; //buf[5] buf[6]
        angleNow.data[i]    = buf[i + 7]; //buf[7] buf[8]
    }
 
    // 锟斤拷取锟斤拷锟狡憋拷志位
    ctrlFlag = buf[9];
    
    Left_v  =-leftVelNow.d;
    Right_v =-rightVelNow.d;
    Angle   =angleNow.d;
 
    return true;
}
/**
 * @brief Get the Crc8 object (unuse)
 * 
 * @param ptr 
 * @param len 
 * @return unsigned char 
 */
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