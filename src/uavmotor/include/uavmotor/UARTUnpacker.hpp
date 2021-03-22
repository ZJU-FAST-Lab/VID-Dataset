
#ifndef __UARTUnpacker_hpp
#define __UARTUnpacker_hpp

#include "ros/ros.h"
#include "vector"

namespace UARTUnpacker
{

#define HD1_VALUE 0xa5
#define HD2_VALUE 0x12

typedef enum
{
    HD_READY,
    WAIT_HD1,
    WAIT_HD2,
    WAIT_LEN,
    WAIT_RANDOM,
    WAIT_CHECKHD
} header_state_e;

typedef enum
{
    DATA_READY,
    WAIT_HD_READY,
    CNT_DATA,
    WAIT_CHECKALL
} pack_state_e;


static const uint8_t crc_table[] =
{
    0x00,0x31,0x62,0x53,0xc4,0xf5,0xa6,0x97,0xb9,0x88,0xdb,0xea,0x7d,0x4c,0x1f,0x2e,
    0x43,0x72,0x21,0x10,0x87,0xb6,0xe5,0xd4,0xfa,0xcb,0x98,0xa9,0x3e,0x0f,0x5c,0x6d,
    0x86,0xb7,0xe4,0xd5,0x42,0x73,0x20,0x11,0x3f,0x0e,0x5d,0x6c,0xfb,0xca,0x99,0xa8,
    0xc5,0xf4,0xa7,0x96,0x01,0x30,0x63,0x52,0x7c,0x4d,0x1e,0x2f,0xb8,0x89,0xda,0xeb,
    0x3d,0x0c,0x5f,0x6e,0xf9,0xc8,0x9b,0xaa,0x84,0xb5,0xe6,0xd7,0x40,0x71,0x22,0x13,
    0x7e,0x4f,0x1c,0x2d,0xba,0x8b,0xd8,0xe9,0xc7,0xf6,0xa5,0x94,0x03,0x32,0x61,0x50,
    0xbb,0x8a,0xd9,0xe8,0x7f,0x4e,0x1d,0x2c,0x02,0x33,0x60,0x51,0xc6,0xf7,0xa4,0x95,
    0xf8,0xc9,0x9a,0xab,0x3c,0x0d,0x5e,0x6f,0x41,0x70,0x23,0x12,0x85,0xb4,0xe7,0xd6,
    0x7a,0x4b,0x18,0x29,0xbe,0x8f,0xdc,0xed,0xc3,0xf2,0xa1,0x90,0x07,0x36,0x65,0x54,
    0x39,0x08,0x5b,0x6a,0xfd,0xcc,0x9f,0xae,0x80,0xb1,0xe2,0xd3,0x44,0x75,0x26,0x17,
    0xfc,0xcd,0x9e,0xaf,0x38,0x09,0x5a,0x6b,0x45,0x74,0x27,0x16,0x81,0xb0,0xe3,0xd2,
    0xbf,0x8e,0xdd,0xec,0x7b,0x4a,0x19,0x28,0x06,0x37,0x64,0x55,0xc2,0xf3,0xa0,0x91,
    0x47,0x76,0x25,0x14,0x83,0xb2,0xe1,0xd0,0xfe,0xcf,0x9c,0xad,0x3a,0x0b,0x58,0x69,
    0x04,0x35,0x66,0x57,0xc0,0xf1,0xa2,0x93,0xbd,0x8c,0xdf,0xee,0x79,0x48,0x1b,0x2a,
    0xc1,0xf0,0xa3,0x92,0x05,0x34,0x67,0x56,0x78,0x49,0x1a,0x2b,0xbc,0x8d,0xde,0xef,
    0x82,0xb3,0xe0,0xd1,0x46,0x77,0x24,0x15,0x3b,0x0a,0x59,0x68,0xff,0xce,0x9d,0xac
};

uint8_t crc8(uint8_t *ptr, uint8_t len) 
{
    uint8_t crc = 0x00;

    while (len--)
    {
        crc = crc_table[crc ^ (*ptr)];
        ptr++;
    }
    return (crc);
}

class header
{
public:

    uint8_t len;
    uint8_t random;
    uint8_t hdcheckbyte;
    header_state_e state;

    uint8_t checkheader(void)
    {
        uint8_t packhd[16] = {0};
        packhd[0] = HD1_VALUE;
        packhd[1] = HD2_VALUE;
        packhd[2] = len;
        packhd[3] = random;
        
        return crc8(packhd, 4);
    }

    void reset(void)
    {
        state = WAIT_HD1;
        len = 0;
        random = 0;
        hdcheckbyte = 0;
    }

    header_state_e readstate(void)
    {
        return state;
    }
    
    uint8_t readlen(void)
    {
        return len;
    }

    header(void)
    {
        reset();
    }

    std::vector<uint8_t> getvec(void)
    {
        std::vector<uint8_t> vec(5);
        vec[0] = HD1_VALUE;
        vec[1] = HD2_VALUE;
        vec[2] = len;
        vec[3] = random;
        vec[4] = hdcheckbyte;
        return vec;
    }

    header_state_e push(uint8_t newbyte)
    {
        switch (state)
        {
        case HD_READY:
            if(newbyte == HD1_VALUE)
            {
                state = WAIT_HD2;
            }
            else
            {
                state = WAIT_HD1;
            }
            
            break;
        case WAIT_HD1:
            if(newbyte == HD1_VALUE)
            {
                state = WAIT_HD2;
            }
            break;
        case WAIT_HD2:
            if(newbyte == HD2_VALUE)
            {
                state = WAIT_LEN;
            }
            break;
        case WAIT_LEN:
            len = newbyte;
            state = WAIT_RANDOM;
            break;
        case WAIT_RANDOM:
            random = newbyte;
            state = WAIT_CHECKHD;
            break;
        case WAIT_CHECKHD:
            hdcheckbyte = newbyte;
            if(hdcheckbyte == checkheader())
            {
                state = HD_READY;
            }
            break;
        default:
            state = WAIT_HD1;
            break;
        }
        return state;
    }
};

class unpacker
{
    std::vector<uint8_t> vec;
    uint8_t len;
    pack_state_e state;
    header hd;

    uint8_t checkall(void)
    {
        std::vector<uint8_t> allvec = hd.getvec();
        allvec.insert(allvec.end(), vec.begin(), vec.end());

        uint8_t allpack[256];
        allpack[0] = HD1_VALUE;
        allpack[1] = HD2_VALUE;
        allpack[2] = hd.len;
        allpack[3] = hd.random;
        allpack[4] = hd.hdcheckbyte;

        for(int i = 0; i <= len - 1; i++)
        {
            allpack[5 + i] = vec[i];
        }

        return crc8(allpack, len + 5);
    }

public:

    void reset(void)
    {
        vec.clear();
        state = WAIT_HD_READY;
    }

    unpacker(void)
    {
        hd.reset();
        reset();
    }

    pack_state_e push(uint8_t newbyte)
    {
        if(hd.push(newbyte) == HD_READY)
        {
            len = hd.readlen();
            vec.clear();
            state = CNT_DATA;
        }
        else
        {
            switch (state)
            {
            case DATA_READY:
                reset();
            case WAIT_HD_READY:
                break;
            case CNT_DATA:
                vec.push_back(newbyte);
                if(vec.size() >= len)
                {
                    state = WAIT_CHECKALL;
                }
                break;
            case WAIT_CHECKALL:
                if(newbyte == checkall())
                {
                    state = DATA_READY;
                }
                break;
            default:
                reset();
                break;
            }
        }
        return state;
    }

    std::vector<uint8_t> readbuff(void)
    {
        return vec;
    }

};

}

#endif





