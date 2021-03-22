
#ifndef __DataClassifier_hpp
#define __DataClassifier_hpp

#include "ros/ros.h"
#include "vector"
#include <uavmotor/m3508.h>
#include <uavmotor/motordata.h>
#include <uavmotor/pulsecap.h>


namespace DataClassifier
{

typedef enum
{
    PACK_NO_ID          = 0x00,
    MOTOR_MSG           = 0xf0,
    CTRL_CURRENT_MSG    = 0xf1,
    TRG_RPM_MSG         = 0xf2,
    PULSE_CAP_MSG       = 0xa0
} pack_id_e;

#define MOTOR_MSG_LEN       11
#define CTRL_CURRENT_LEN    14
#define TRG_RPM_LEN         14
#define PULSE_CAP_LEN       7


pack_id_e packid(std::vector<uint8_t> &buff)
{
    if(buff[0] == MOTOR_MSG && buff.size() == MOTOR_MSG_LEN)
    {
        return MOTOR_MSG;
    }
    else if(buff[0] == CTRL_CURRENT_MSG && buff.size() == CTRL_CURRENT_LEN)
    {
        return CTRL_CURRENT_MSG;
    }
    else if(buff[0] == TRG_RPM_MSG && buff.size() == TRG_RPM_LEN)
    {
        return TRG_RPM_MSG;
    }
    else if(buff[0] == PULSE_CAP_MSG && buff.size() == PULSE_CAP_LEN)
    {
        return PULSE_CAP_MSG;
    }    
    else
    {
        return PACK_NO_ID;
    }
}


void GetMotorMsg(uavmotor::m3508 *pmotor, std::vector<uint8_t> &buff)
{
    pmotor->header.stamp = ros::Time::now();
    pmotor->hwts.sec = buff[1] | (buff[2] << 8);
    pmotor->hwts.nsec = (buff[3] | (buff[4] << 8) | (buff[5] << 16)) * 1000;
    pmotor->id = buff[6];
    pmotor->current = buff[7] | (buff[8] << 8);
    pmotor->rpm = buff[9] | (buff[10] << 8);
}

void GetCurrentMsg(uavmotor::motordata *pcurrent, std::vector<uint8_t> &buff)
{
    pcurrent->header.stamp = ros::Time::now();
    pcurrent->hwts.sec = buff[1] | (buff[2] << 8);
    pcurrent->hwts.nsec = (buff[3] | (buff[4] << 8) | (buff[5] << 16)) * 1000;
    pcurrent->data[0] = buff[6] | (buff[7] << 8);
    pcurrent->data[1] = buff[8] | (buff[9] << 8);
    pcurrent->data[2] = buff[10] | (buff[11] << 8);
    pcurrent->data[3] = buff[12] | (buff[13] << 8);
}

void GetRPMMsg(uavmotor::motordata *prpm, std::vector<uint8_t> &buff)
{
    prpm->header.stamp = ros::Time::now();
    prpm->hwts.sec = buff[1] | (buff[2] << 8);
    prpm->hwts.nsec = (buff[3] | (buff[4] << 8) | (buff[5] << 16)) * 1000;
    prpm->data[0] = buff[6] | (buff[7] << 8);
    prpm->data[1] = buff[8] | (buff[9] << 8);
    prpm->data[2] = buff[10] | (buff[11] << 8);
    prpm->data[3] = buff[12] | (buff[13] << 8);    
}

void GetPulseCapMsg(uavmotor::pulsecap *pcap, std::vector<uint8_t> &buff)
{
    pcap->header.stamp = ros::Time::now();
    pcap->hwts.sec = buff[1] | (buff[2] << 8);
    pcap->hwts.nsec = (buff[3] | (buff[4] << 8) | (buff[5] << 16)) * 1000;
    pcap->id = buff[6];
}

}

#endif





