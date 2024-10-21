
#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include<stdint.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <iostream>
#include "CRC_Check.h"
using namespace std;

#define TRUE 1
#define FALSE 0

#define RDATA_LENGTH 16

//串口的相关参数
#define BAUDRATE 460800//波特率
#define UART_DEVICE "/dev/ttyUSB0"//默认的串口名称

//C_lflag
#define ECHOFLAGS (ECHO | ECHOE | ECHOK | ECHONL)

typedef union
{
    float f;
    unsigned char c[4];
} float2uchar;

//字节数为2的uchar数据类型
typedef union
{
    int16_t d;
    unsigned char c[2];
} int16uchar;

typedef enum {
    AIM_OFF=0x00,
    AIM_ON=0x01,
    AIM_SMALL_BUFF=0x02,
    AIM_BIG_BUFF=0x03,
    AIM_ANTOP=0x04,
    AIM_ANDF=0x05
}vision_cmd_e;
typedef struct{
    uint8_t SOF;
    uint8_t mode;
    uint8_t CRC8;
    float yaw_angle;
    float pitch_angle;
    uint8_t my_color;
    uint8_t blood_0;
    uint8_t blood_1;
    uint8_t blood_2;
    uint8_t blood_3;
    uint8_t blood_4;
//    uint8_t blood_5;
//    uint8_t blood_6;
//    uint8_t blood_7;
//    uint8_t blood_8;
//    uint8_t size_3;
//    uint8_t size_4;
//    uint8_t size_5;
//    uint8_t is_change_target;
//    uint8_t is_hit_outpost;
//    uint8_t is_blood_first;
//    uint8_t game_progress;
//    uint16_t chassis_buffer;
    uint16_t CRC16;
}__attribute__((__packed__)) vision_tx_info_t;
typedef struct
{
    uint8_t SOF;
    uint8_t mode;
    uint8_t CRC8;
    float2uchar pitch_angle;
    float2uchar yaw_angle;
    uint8_t is_find_target;
    uint8_t armor_num;
    uint8_t is_find_buff;
    uint8_t is_hit_enable;
    uint8_t shoot_type;
    uint8_t is_spin;
    uint16_t CRC16;
    void reset();
    void make_data_safe();
}__attribute__((__packed__)) vision_rx_info_t;
//字节数为4的结构体


//用于保存目标相关角度和距离信息及瞄准情况
typedef struct
{
    float2uchar pitch_angle;//俯仰角
    float2uchar yaw_angle;//偏航角
    float2uchar dis;//目标距离
    int drop_frame;
    int isFindTarget;
    int isfindDafu;
    int nearFace;
    int anti_top;
    int anti_top_change_armor;
    int _mode;

    void reset();
    void make_data_safe();
} VisionData;



class SerialPort
{
private:
    int fd; //串口号
    int speed, databits, stopbits, parity;
    unsigned char rdata[4096]; //raw_data
    unsigned char Tdata[30];  //transfrom data
    int drop_cnt;
    void set_Brate();
    int set_Bit();

public:
    int result;
    SerialPort();
    SerialPort(char *);
    bool initSerialPort();
    bool get_Mode(int &mode, int &shoot_speed, int &my_color,
                  double &gyro_yaw, double &gyro_pitch, int &anti_flag);
    void TransformData(const VisionData &data); //主要方案
    void send();
    void closePort();
    void TransformDataFirst(int Xpos, int Ypos, int dis);//方案1
    bool reinitSerialPort(char *portpath);
    bool my_get_Mode(vision_tx_info_t& cardata);
    void my_transform_data(const vision_rx_info_t& visiondata);
//  int set_disp_mode(int);
//  void TransformTarPos(const VisionData &data);
};


#endif //SERIALPORT_H
