#include <ros/ros.h>

#include <djiros/ImuWithHwTimeStamp.h>
#include <djiros/Pulse.h>
#include <sensor_msgs/Imu.h>

#include <uavmotor/motordata.h>
#include <uavmotor/m3508.h>
#include <uavmotor/pulsecap.h>

#include <sensor_msgs/Image.h>
#include <mav_msgs/Actuators.h>

#include <list>

#define RPMFAC  5
#define RPM_LEN (2 * RPMFAC + 1)

std::list<uavmotor::m3508> motor[4];

void m3508m1Callback(const uavmotor::m3508::ConstPtr& pmsg)
{
    motor[0].push_back(*pmsg);
    if(motor[0].size() > RPM_LEN)
    {
        motor[0].pop_front();
    }
}

void m3508m2Callback(const uavmotor::m3508::ConstPtr& pmsg)
{
    motor[1].push_back(*pmsg);
    if(motor[1].size() > RPM_LEN)
    {
        motor[1].pop_front();
    }
}

void m3508m3Callback(const uavmotor::m3508::ConstPtr& pmsg)
{
    motor[2].push_back(*pmsg);
    if(motor[2].size() > RPM_LEN)
    {
        motor[2].pop_front();
    }
}

void m3508m4Callback(const uavmotor::m3508::ConstPtr& pmsg)
{
    motor[3].push_back(*pmsg);
    if(motor[3].size() > RPM_LEN)
    {
        motor[3].pop_front();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "rpmcpnvert");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(100);

    ros::Subscriber sub_m3508[4];
    sub_m3508[0] = n.subscribe("/synced/m3508_m1", 10, m3508m1Callback, ros::TransportHints().tcpNoDelay());
    sub_m3508[1] = n.subscribe("/synced/m3508_m2", 10, m3508m2Callback, ros::TransportHints().tcpNoDelay());
    sub_m3508[2] = n.subscribe("/synced/m3508_m3", 10, m3508m3Callback, ros::TransportHints().tcpNoDelay());
    sub_m3508[3] = n.subscribe("/synced/m3508_m4", 10, m3508m4Callback, ros::TransportHints().tcpNoDelay());

    ros::Publisher pub_rpm = n.advertise<uavmotor::motordata>("/synced/allrpm", 10);

    ros::Publisher pub_vidrpm = n.advertise<mav_msgs::Actuators>("/synced/vidrpm", 10);


    while(ros::ok())
    {
        ros::spinOnce();

        if
        (       motor[0].size() == RPM_LEN
            &&  motor[1].size() == RPM_LEN
            &&  motor[2].size() == RPM_LEN
            &&  motor[3].size() == RPM_LEN
        )
        {

            std::vector<int16_t> newmotor[4];
            int16_t rpm[4];
            for(int i = 0; i <= 3; i++)
            {
                std::list<uavmotor::m3508>::iterator it = motor[i].begin();
                while (1)
                {
                    newmotor[i].push_back(it->rpm);
                    if(it == motor[i].end())
                    {
                        break;
                    }
                    else
                    {
                        it++;
                    }
                }
                
                std::sort(newmotor[i].begin(), newmotor[i].end());
                
                rpm[i] = newmotor[i][RPMFAC];
            }

            // printf("%.3f ", motor[0].back().header.stamp.toSec());
            // printf("%.3f ", motor[1].back().header.stamp.toSec());
            // printf("%.3f ", motor[2].back().header.stamp.toSec());
            // printf("%.3f ", motor[3].back().header.stamp.toSec());
            // printf("\r\n");
            uavmotor::motordata msg;
            msg.header.stamp = motor[0].back().header.stamp;
            msg.hwts = msg.header.stamp;
            msg.data[0] = rpm[0];
            msg.data[1] = rpm[1];
            msg.data[2] = rpm[2];
            msg.data[3] = rpm[3];
            pub_rpm.publish(msg);

            mav_msgs::Actuators vidmsg;
            vidmsg.header.stamp = msg.header.stamp;
            vidmsg.angular_velocities.push_back(msg.data[0]);
            vidmsg.angular_velocities.push_back(msg.data[1]);
            vidmsg.angular_velocities.push_back(msg.data[2]);
            vidmsg.angular_velocities.push_back(msg.data[3]);
            pub_vidrpm.publish(vidmsg);
        }



        loop_rate.sleep();
    }

    

    ros::spin();

    return 0;
}