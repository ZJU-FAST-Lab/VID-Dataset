

#include <ros/ros.h>  
#include <serial/serial.h>
#include <uavmotor/UARTUnpacker.hpp>
#include <uavmotor/DataClassifier.hpp>


serial::Serial sp;

std::string serial_port;
int looprate = 1000;
int bandrate = 921600;


void node_param_init(ros::NodeHandle &n)
{
    n.getParam("serial_port", serial_port);
    n.getParam("looprate", looprate);
    n.getParam("bandrate", bandrate);
}

bool serial_port_init(std::string serial_port)
{
    ros::NodeHandle node; 

    sp.setPort(serial_port); 
    sp.setBaudrate(bandrate); 
    serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
    sp.setTimeout(to);

    try 
    { 
        sp.open(); 
    } 
    catch(serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port.");  
        return false; 
    } 

    if(sp.isOpen())
    {
        ROS_INFO_STREAM("opened.");
    }
    else
    {
        return false;
    }
    return true;
}


bool serial_offline = false;
bool serial_port_check()
{
    // ROS_ERROR_STREAM("serial port is offline.");
    if(!serial_offline)
    {
        try 
        { 
            sp.available();
        } 
        catch(serial::IOException& e) 
        { 
            serial_offline = true;
            sp.close();
            sp.setPort(serial_port); 
            sp.setBaudrate(bandrate); 
            serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
            sp.setTimeout(to);
            ROS_ERROR_STREAM("serial lost!");  
        }
    }
    if(serial_offline)
    {
        try 
        { 
            sp.open(); 
        } 
        catch(serial::IOException& e) 
        { 
            ROS_ERROR_STREAM("Unable to reopen port. Ignore cmd this time.");  
            return false; 
        } 
        serial_offline = false;
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "m100withm3508");
    ros::NodeHandle n("~");
    node_param_init(n);

    ros::Publisher pub_m3508[4];
    pub_m3508[0] = n.advertise<uavmotor::m3508>("m3508_m1", 10);
    pub_m3508[1] = n.advertise<uavmotor::m3508>("m3508_m2", 10);
    pub_m3508[2] = n.advertise<uavmotor::m3508>("m3508_m3", 10);
    pub_m3508[3] = n.advertise<uavmotor::m3508>("m3508_m4", 10);

    ros::Publisher pub_ctrl_current = n.advertise<uavmotor::motordata>("ctrl_current", 10);
    ros::Publisher pub_target_rpm = n.advertise<uavmotor::motordata>("target_rpm", 10);

    ros::Publisher pub_pps = n.advertise<uavmotor::pulsecap>("cap_n3pps", 10);
    ros::Publisher pub_camera = n.advertise<uavmotor::pulsecap>("cap_camerapulse", 10);

    serial_port_init(serial_port);
    ros::Rate loop_rate(looprate);


    UARTUnpacker::unpacker unpk;
    UARTUnpacker::header hd;
    while(ros::ok())
    {
        ros::spinOnce();
        if(serial_port_check())
        {
            std::string databuff;
            size_t datalen = sp.available(); 
            if(datalen != 0) 
            {
                // ros::Time t0 = ros::Time::now();

                databuff = sp.read(datalen);
                
                for(uint i = 0; i <= datalen - 1; i++)
                {
                    
                    // for(uint i = 0; i <= datalen - 1; i++)
                    // {
                    //     printf("%02x ", (uint8_t)databuff[i]);
                    // }
                    // printf("\r\n");                    

                    

                    // if(hd.push(databuff[i]) == UARTUnpacker::HD_READY)
                    // {
                    //     printf("%02x %02x %02x\r\n", hd.len, hd.random, hd.hdcheckbyte);
                    // }

                    if(unpk.push(databuff[i]) == UARTUnpacker::DATA_READY)
                    {
                        
                        std::vector<uint8_t> vec = unpk.readbuff();
                        
                        // static int n = 0;
                        // printf("n = %d  len = %d |||\t", n++, (int)vec.size());
                        // for(size_t i = 0; i <= vec.size() - 1; i++)
                        // {
                        //     printf("%02x ", (uint8_t)vec[i]);
                        // }
                        // printf("   ");

                        DataClassifier::pack_id_e id = DataClassifier::packid(vec);
                        uavmotor::m3508 msg_m3508;
                        uavmotor::motordata msg_motor;
                        uavmotor::pulsecap msg_cap;
                        switch (id)
                        {
                        case DataClassifier::MOTOR_MSG:
                            DataClassifier::GetMotorMsg(&msg_m3508, vec);
                            if(msg_m3508.id >=1 && msg_m3508.id <= 4)
                            {
                                pub_m3508[msg_m3508.id - 1].publish(msg_m3508);
                            }
                            break;
                        case DataClassifier::CTRL_CURRENT_MSG:
                            DataClassifier::GetCurrentMsg(&msg_motor, vec);
                            pub_ctrl_current.publish(msg_motor);
                            break;
                        case DataClassifier::TRG_RPM_MSG:
                            DataClassifier::GetRPMMsg(&msg_motor, vec);
                            pub_target_rpm.publish(msg_motor);
                            break;
                        case DataClassifier::PULSE_CAP_MSG:
                            DataClassifier::GetPulseCapMsg(&msg_cap, vec);
                            if(msg_cap.id == 1)
                            {
                                pub_pps.publish(msg_cap);
                            }
                            else
                            {
                                pub_camera.publish(msg_cap);
                            }
                            

                        default:
                            break;
                        }


                    }
                }
                // ros::Time tf = ros::Time::now();
                // printf("%d %d ns\r\n", datalen, (tf - t0).toNSec());
            }
        }
        loop_rate.sleep();
    }
    
    sp.close();
    return 0;
}











