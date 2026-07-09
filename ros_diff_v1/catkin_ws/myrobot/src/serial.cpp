/**
 * @file serial.cpp
 * @author thoelc
 * @brief 基于boost::asio实现的串口通信，替代了RosSerial直接控制
 * @version 0.1
 * @date 2023-11-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "serial.h"

using namespace std;
using namespace boost::asio;

// 通信用的串口对象
boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev, "/dev/chasis");
boost::system::error_code err;

/*******************************************************************************
 *  定义帧头和帧尾数据报
*******************************************************************************/
const unsigned char header[2] = {0x55, 0xaa};
const unsigned char ender[2] = {0x0d, 0x0a};

union sendData
{
    short speed;
    unsigned char data_speed[2];
} leftVelSet, rightVelSet;

// 使用联合体定义数据包，大小端转换（-32767 - +32768)
union receiveData
{
    short d;
    unsigned char data[2];
} leftVelNow, rightVelNow, angleNow;

/*******************************************************************************
串口初始化，波特率115200
*******************************************************************************/
void serialInit()
{
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));
}

/**
 * @brief 写速度函数，用于发送给下位机控制器
 *
 * @param Left_v 左轮速度
 * @param Right_v 右轮速度
 * @param ctrlFlag 控制标志位
 */
void writeSpeed(double Left_v, double Right_v, unsigned char ctrlFlag)
{
    unsigned char buf[13] = {0};
    int i, length = 0;

    leftVelSet.speed = Left_v;   // mm/s
    rightVelSet.speed = Right_v; // mm/s

    // 写入帧头
    for (i = 0; i < 2; i++)
        buf[i] = header[i];

    // 设置用户数据内容和速度
    length = 5;
    buf[2] = length;
    for (i = 0; i < 2; i++)
    {
        buf[i + 3] = leftVelSet.data_speed[i];
        buf[i + 5] = rightVelSet.data_speed[i];
    }
    // 预留控制指针
    buf[3 + length - 1] = ctrlFlag;

    buf[3 + length] = ender[0];
    buf[3 + length + 1] = ender[1];

    // 通过串口发送数据
    boost::asio::write(sp, boost::asio::buffer(buf));
}

/**
 * @brief 读速度函数
 *
 * @param Left_v 左轮速度
 * @param Right_v 右轮速度
 * @param Angle 角度
 * @param ctrlFlag 控制标志位
 * @return true 读取成功
 * @return false 读取失败
 */
bool readSpeed(double& Left_v, double& Right_v, double& Angle, unsigned char& ctrlFlag)
{
    char i, length = 0;
    unsigned char checkSum;
    unsigned char buf[1024] = {0};

    try
    {
        boost::asio::streambuf response;
        boost::asio::read_until(sp, response, "\r\n", err);
        copy(istream_iterator<unsigned char>(istream(&response) >> noskipws),
             istream_iterator<unsigned char>(),
             buf);
    }
    catch (boost::system::system_error& err)
    {
        ROS_INFO("read_until error");
    }

    // 校验消息头
    if (buf[0] != header[0] || buf[1] != header[1])
    {
        ROS_ERROR("Received message header error!");
        return false;
    }

    // 数据长度
    length = buf[2];

    // 校验消息校验和（CRC8）
    // checkSum = getCrc8(buf, 3 + length);
    // if (checkSum != buf[3 + length])
    // {
    //     ROS_ERROR("Received data check sum error!");
    //     return false;
    // }

    // 读取速度值
    for (i = 0; i < 2; i++)
    {
        leftVelNow.data[i]  = buf[i + 3];
        rightVelNow.data[i] = buf[i + 5];
        angleNow.data[i]    = buf[i + 7];
    }

    // 读取控制标志位
    ctrlFlag = buf[9];

    Left_v  = -leftVelNow.d;
    Right_v = -rightVelNow.d;
    Angle   = angleNow.d;

    return true;
}

/**
 * @brief 计算CRC8校验和（未使用）
 *
 * @param ptr 数据指针
 * @param len 数据长度
 * @return unsigned char 校验和
 */
unsigned char getCrc8(unsigned char* ptr, unsigned short len)
{
    unsigned char crc;
    unsigned char i;
    crc = 0;
    while (len--)
    {
        crc ^= *ptr++;
        for (i = 0; i < 8; i++)
        {
            if (crc & 0x01)
                crc = (crc >> 1) ^ 0x8C;
            else
                crc >>= 1;
        }
    }
    return crc;
}