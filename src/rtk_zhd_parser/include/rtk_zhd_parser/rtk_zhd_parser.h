#ifndef RTK_ZHD_PARSER_H
#define RTK_ZHD_PARSER_H

#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include "rtk_zhd_parser/serial.h"

#define RTK_ZHD_UNLOGALL        "unlogall com1\r\n"
#define RTK_ZHD_CUSTOM_5HZ      "zhd log com1 gpsdata ontime 0.2\r\n"
#define RTK_ZHD_CUSTOM_10HZ     "zhd log com1 gpsdata ontime 0.1\r\n"
#define RTK_ZHD_ENABLE_INS      "zhd ins enable\r\n"
#define RTK_ZHD_SAVECONFIG      "saveconfig\r\n"
#define RTK_ZHD_MSG_FREQ        10.0
#define RTK_ZHD_HEAD1           0xAA
#define RTK_ZHD_HEAD2           0x33
#define RTK_ZHD_FRAME_SIZE      138

#define PI_LOCAL                3.141592653589793


enum RTK_ZHD_PARSING_STATE
{
    RTK_ZHD_PARSING_UNINIT,
    RTK_ZHD_PARSING_GOT_HEAD1,
    RTK_ZHD_PARSING_GOT_HEAD2,
};

#pragma pack(push, 1)
struct RTK_ZHD_Data
{
    unsigned char head[2];                 //2 bytes deviation 0
    unsigned short int version;            //2 bytes deviation 2
    unsigned short int length;             //2 bytes deviation 4
    unsigned short int freq;               //2 bytes deviation 6
    float time_utc;                        //4 bytes deviation 8
    unsigned short int year_utc;           //2 bytes deviation 12
    unsigned short int month_utc;          //2 bytes deviation 14
    unsigned short int day_utc;            //2 bytes deviation 16
    unsigned short int hour_utc;           //2 bytes deviation 18
    unsigned short int min_utc;            //2 bytes deviation 20
    unsigned short int sec_utc;            //2 bytes deviation 22
    double latitude;                       //8 bytes deviation 24
    double longitude;                      //8 bytes deviation 32
    double altitude;                       //8 bytes deviation 40
    float eph;                             //4 bytes deviation 48
    float epv;                             //4 bytes deviation 52
    float vel_ground_m_s;                  //4 bytes deviation 56
    float angle_tracktrue;                 //4 bytes deviation 60
    float angle_heading;                   //4 bytes deviation 64
    float angle_pitch;                     //4 bytes deviation 68
    double vel_n_m_s;                      //8 bytes deviation 72
    double vel_e_m_s;                      //8 bytes deviation 80
    double vel_u_m_s;                      //8 bytes deviation 88
    unsigned short int satellites_used;    //2 bytes deviation 96
    unsigned short int satellites_track;   //2 bytes deviation 98
    float vel_ned_valid;                   //4 bytes deviation 100
    unsigned short int fix_type;           //2 bytes deviation 104
    float angle_postype;                   //4 bytes deviation 106
    float head_deviation;                  //4 bytes deviation 110
    unsigned short int ins_state;          //2 bytes deviation 114
    double gnss_alt_delta ;                //8 bytes deviation 116
    double ellipsoidal_h;                  //8 bytes deviation 124
    unsigned short int diff_age;           //2 bytes deviation 132
    unsigned char reserve[2];              //2 bytes deviation 134
    unsigned short int checksum;           //2 bytes deviation 136
};                                         //total 138 bytes
#pragma pack(pop)

class RTK_ZHD_Parser
{
public:
    RTK_ZHD_Parser(ros::NodeHandle n);
    ~RTK_ZHD_Parser();
    
    ros::NodeHandle nh, nh_;
    ros::Publisher pubGPS;
    ros::Publisher pubYaw;
    ros::Publisher pubVel;
    ros::Publisher pubSatellitesStatus;
    std::string RTK_ZHD_GPS_TOPIC;
    std::string RTK_ZHD_YAW_TOPIC;
    std::string RTK_ZHD_VELOCITY_TOPIC;
    std::string RTK_ZHD_SATELLITES_TOPIC;
    std::string RTK_ZHD_PORT_PATH;
    int RTK_ZHD_BAUD_RATE;
    double RTK_ZHD_IDLE_FREQ;
    int RTK_ZHD_NUM_TO_IDLE;
    
    serial_t serial;
    
    void parseInit();
    void runParser();
    int parseChar(uint8_t curByte, RTK_ZHD_Data &packet);
    void handleData(RTK_ZHD_Data &packet);
    void checkByteSum(uint8_t curByte);
    RTK_ZHD_PARSING_STATE parseState;
    unsigned rxCount;
    uint16_t rxXor;
};

#endif
