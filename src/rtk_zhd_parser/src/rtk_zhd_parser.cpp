#include "rtk_zhd_parser/rtk_zhd_parser.h"

using namespace std;
using namespace ros;

RTK_ZHD_Parser::RTK_ZHD_Parser(ros::NodeHandle n): nh(n), nh_("~")
{
    nh_.getParam("RTK_ZHD_GPS_TOPIC", RTK_ZHD_GPS_TOPIC);
    nh_.getParam("RTK_ZHD_YAW_TOPIC", RTK_ZHD_YAW_TOPIC);
    nh_.getParam("RTK_ZHD_VELOCITY_TOPIC", RTK_ZHD_VELOCITY_TOPIC);
    nh_.getParam("RTK_ZHD_SATELLITES_TOPIC", RTK_ZHD_SATELLITES_TOPIC);
    nh_.getParam("RTK_ZHD_PORT_PATH", RTK_ZHD_PORT_PATH);
    nh_.getParam("RTK_ZHD_BAUD_RATE", RTK_ZHD_BAUD_RATE);
    nh_.getParam("RTK_ZHD_IDLE_FREQ", RTK_ZHD_IDLE_FREQ);
    nh_.getParam("RTK_ZHD_NUM_TO_IDLE", RTK_ZHD_NUM_TO_IDLE);
    pubGPS = nh.advertise<sensor_msgs::NavSatFix>(RTK_ZHD_GPS_TOPIC, 1);
    pubYaw = nh.advertise<std_msgs::Float64MultiArray>(RTK_ZHD_YAW_TOPIC, 1);
    pubVel = nh.advertise<geometry_msgs::Vector3Stamped>(RTK_ZHD_VELOCITY_TOPIC, 1);
    pubSatellitesStatus = nh.advertise<std_msgs::Int64MultiArray>(RTK_ZHD_SATELLITES_TOPIC, 1);
    if (serial_open(&serial, RTK_ZHD_PORT_PATH.c_str(), RTK_ZHD_BAUD_RATE) < 0)
    {
        ROS_ERROR("Serial port of RTK cannot be opened!");
        ros::shutdown();
        exit(0);
    }
    
//Initiatively initializing may purge the localization process, hence commented.
//Never use these unless necessary !!!
/*
    uint8_t unLogAll[] = RTK_ZHD_UNLOGALL;
    uint8_t customFreq[] = RTK_ZHD_CUSTOM_10HZ;
    uint8_t enINS[] = RTK_ZHD_ENABLE_INS;
    uint8_t savConfig[] = RTK_ZHD_SAVECONFIG;
    
    serial_write(&serial, unLogAll, sizeof(unLogAll)-1);
    usleep(10000);
    serial_write(&serial, customFreq, sizeof(customFreq)-1);
    usleep(10000);
    serial_write(&serial, enINS, sizeof(enINS)-1);
    usleep(10000);
    serial_write(&serial, savConfig, sizeof(savConfig)-1);
    usleep(10000);
*/
    
    parseInit();
}

RTK_ZHD_Parser::~RTK_ZHD_Parser()
{
    serial_close(&serial);
}

void RTK_ZHD_Parser::parseInit()
{
    rxCount = 0;
    rxXor = 0;
    parseState = RTK_ZHD_PARSING_UNINIT;
}

void RTK_ZHD_Parser::runParser()
{
    ros::Rate loop_rate(2.0*RTK_ZHD_MSG_FREQ);
    ros::Rate idle_loop_rate(RTK_ZHD_IDLE_FREQ);
    int idle_check_num = 0;
    bool idle = false;
    uint8_t buffer[RTK_ZHD_FRAME_SIZE];
    RTK_ZHD_Data packet;
    
    int count = 0;
    while(ros::ok())
    {
        int bytesReceived = serial_read(&serial, buffer, sizeof(buffer), 0);
        idle_check_num = bytesReceived == 0 ? (idle_check_num + 1) : 0;
        idle = idle_check_num >= RTK_ZHD_NUM_TO_IDLE;
        while (count < bytesReceived)
        {
            if (parseChar(buffer[count++], packet) > 0)
            {
                handleData(packet);
            }
        }
        count = 0;
        if(idle)
        {
            parseInit();
            idle_loop_rate.sleep();
        }
        else
            loop_rate.sleep();
        ros::spinOnce();
    }
}

int RTK_ZHD_Parser::parseChar(uint8_t curByte, RTK_ZHD_Data& packet)
{
    int ret = 0;

    if (parseState == RTK_ZHD_PARSING_UNINIT)
    {
        if (curByte == RTK_ZHD_HEAD1)
        {
            parseState = RTK_ZHD_PARSING_GOT_HEAD1;
            checkByteSum(curByte);
            ((uint8_t*)(&packet))[rxCount] = curByte;
            rxCount++;
        }
    }
    else if(parseState == RTK_ZHD_PARSING_GOT_HEAD1)
    {
        if (curByte == RTK_ZHD_HEAD2)
        {
            parseState = RTK_ZHD_PARSING_GOT_HEAD2;
            checkByteSum(curByte);
            ((uint8_t*)(&packet))[rxCount] = curByte;
            rxCount++;
        }
        else
        {
            parseInit();
        }
    }
    else if(parseState == RTK_ZHD_PARSING_GOT_HEAD2)
    {
        if (rxCount <= (sizeof(packet) - 2 - 1))
        {
            checkByteSum(curByte);
        }

        ((uint8_t*)(&packet))[rxCount] = curByte;
        rxCount++;
        
        if (rxCount >= sizeof(packet))
        {
            /* Compare checksum */
            if (rxXor == packet.checksum)
            {
                ret = 1;
            }
            else
            {
                ret = -1;
            }
            //Reset to decode next packet
            parseInit();
        }
    }

    return ret;
}

void RTK_ZHD_Parser::checkByteSum(uint8_t curByte)
{
    rxXor ^= curByte;
}


void RTK_ZHD_Parser::handleData(RTK_ZHD_Data& packet)
{
    ros::Time curTime = ros::Time::now();
    
    sensor_msgs::NavSatFix GPSdata;
    GPSdata.header.stamp = curTime;
    GPSdata.latitude = packet.latitude;
    GPSdata.longitude = packet.longitude;
    GPSdata.altitude = packet.altitude;
    //GPSdata.status.status : -1 for None Solution, 0 for Bad Solution, 1 for Good Solution.
    GPSdata.status.status = packet.fix_type == 0 ? -1 : 0;
    GPSdata.status.status = packet.fix_type == 4 ? 1 : GPSdata.status.status;
    pubGPS.publish(GPSdata);
    
    std_msgs::Float64MultiArray YawData;
    YawData.layout.data_offset = 0;
    std_msgs::MultiArrayDimension yawDataDim;
    yawDataDim.label = "Yaw-State-Deviation";
    yawDataDim.size = 3;
    yawDataDim.stride = 3;
    YawData.layout.dim.push_back(yawDataDim);
    YawData.data.push_back(packet.angle_heading/180*PI_LOCAL);
    //headStatus : -1 for None Solution, 0 for Bad Solution, 1 for Good Solution.
    double headStatus = packet.angle_postype < 0.1 ? -1 : 0;
    headStatus = (packet.angle_postype < 50.1 && packet.angle_postype > 40.9) ? 1 : headStatus;
    YawData.data.push_back(headStatus);
    YawData.data.push_back(packet.head_deviation);
    pubYaw.publish(YawData);
    
    std_msgs::Int64MultiArray SatellitesData;
    SatellitesData.layout.data_offset = 0;
    std_msgs::MultiArrayDimension satellitesDataDim;
    satellitesDataDim.label = "Num Used - Num Tracked";
    satellitesDataDim.size = 2;
    satellitesDataDim.stride = 2;
    SatellitesData.layout.dim.push_back(satellitesDataDim);
    SatellitesData.data.push_back((int)(packet.satellites_used));
    SatellitesData.data.push_back((int)(packet.satellites_track));
    pubSatellitesStatus.publish(SatellitesData);
    
    geometry_msgs::Vector3Stamped velocityENUData;
    velocityENUData.header.stamp = curTime;
    velocityENUData.header.frame_id = "ENU";
    velocityENUData.vector.x = packet.vel_e_m_s;
    velocityENUData.vector.y = packet.vel_n_m_s;
    velocityENUData.vector.z = packet.vel_u_m_s;
    if(packet.vel_ned_valid > 49.9 && packet.vel_ned_valid < 50.1) pubVel.publish(velocityENUData);    
    
}

