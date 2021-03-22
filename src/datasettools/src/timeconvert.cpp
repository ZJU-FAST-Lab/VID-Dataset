#include <ros/ros.h>

#include <djiros/ImuWithHwTimeStamp.h>
#include <djiros/Pulse.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include <uavmotor/motordata.h>
#include <uavmotor/m3508.h>
#include <uavmotor/pulsecap.h>

#include <sensor_msgs/Image.h>


ros::Publisher pub_imu;
// ros::Publisher pub_gps;
ros::Publisher pub_target_rpm;
ros::Publisher pub_ctrl_current;

ros::Publisher pub_m3508[4];

ros::Publisher pub_depth;
ros::Publisher pub_infra1;
ros::Publisher pub_infra2;

ros::Time last_n3pps_hwts;
ros::Time last_n3pps_rosts;
bool last_n3pps_valid = false;

ros::Time last_mcucap_hwts;
ros::Time last_mcucap_rosts;
bool last_mcucap_valid = false;

ros::Duration mcu2n3_detat;
bool mcu2n3_detat_valid = false;

void imuCallback(const djiros::ImuWithHwTimeStamp::ConstPtr &pmsg)
{
    if(mcu2n3_detat_valid == true)
    {
        sensor_msgs::Imu new_imu_msg;
        new_imu_msg.header.stamp = pmsg->hwts;
        new_imu_msg.orientation = pmsg->orientation;
        new_imu_msg.orientation_covariance = pmsg->orientation_covariance;
        new_imu_msg.angular_velocity = pmsg->angular_velocity;
        new_imu_msg.angular_velocity_covariance = pmsg->angular_velocity_covariance;
        new_imu_msg.linear_acceleration = pmsg->linear_acceleration;
        new_imu_msg.linear_acceleration_covariance = pmsg->linear_acceleration_covariance;
        pub_imu.publish(new_imu_msg);
    }
}

void n3ppsCallback(const djiros::Pulse::ConstPtr& pmsg)
{
    last_n3pps_valid = true;
    last_n3pps_rosts = pmsg->header.stamp;
    last_n3pps_hwts = pmsg->hwts;
    if
    (
            last_mcucap_valid == true
        &&  last_n3pps_valid == true
        &&  fabs((last_mcucap_rosts - last_n3pps_rosts).toSec()) < 0.01
    )
    {
        mcu2n3_detat = last_n3pps_hwts - last_mcucap_hwts;
        mcu2n3_detat_valid = true;
        // printf("n3pps dt = %f sec\r\n", mcu2n3_detat.toSec());
    }    
}

// void n3gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& pmsg)
// {
//     sensor_msgs::NavSatFix new_msg;
//     if(mcu2n3_detat_valid == true)
//     {
//         new_msg = *pmsg;
//         pub_gps.publish(new_msg);
//     }
// }




void ctrlRPMCallback(const uavmotor::motordata::ConstPtr& pmsg)
{
    uavmotor::motordata new_msg;
    if(mcu2n3_detat_valid == true)
    {
        new_msg = *pmsg;
        new_msg.header.stamp = pmsg->hwts + mcu2n3_detat;
        pub_target_rpm.publish(new_msg);
    }

}

void ctrlCurrentCallback(const uavmotor::motordata::ConstPtr& pmsg)
{
    uavmotor::motordata new_msg;
    if(mcu2n3_detat_valid == true)
    {
        new_msg = *pmsg;
        new_msg.header.stamp = pmsg->hwts + mcu2n3_detat;
        pub_ctrl_current.publish(new_msg);
    }
}

void m3508m1Callback(const uavmotor::m3508::ConstPtr& pmsg)
{
    uavmotor::m3508 new_msg;
    if(mcu2n3_detat_valid == true)
    {
        new_msg = *pmsg;
        new_msg.header.stamp = pmsg->hwts + mcu2n3_detat;
        pub_m3508[0].publish(new_msg);
    }
}

void m3508m2Callback(const uavmotor::m3508::ConstPtr& pmsg)
{
    uavmotor::m3508 new_msg;
    if(mcu2n3_detat_valid == true)
    {
        new_msg = *pmsg;
        new_msg.header.stamp = pmsg->hwts + mcu2n3_detat;
        pub_m3508[1].publish(new_msg);
    }
}

void m3508m3Callback(const uavmotor::m3508::ConstPtr& pmsg)
{
    uavmotor::m3508 new_msg;
    if(mcu2n3_detat_valid == true)
    {
        new_msg = *pmsg;
        new_msg.header.stamp = pmsg->hwts + mcu2n3_detat;
        pub_m3508[2].publish(new_msg);
    }
}

void m3508m4Callback(const uavmotor::m3508::ConstPtr& pmsg)
{
    uavmotor::m3508 new_msg;
    if(mcu2n3_detat_valid == true)
    {
        new_msg = *pmsg;
        new_msg.header.stamp = pmsg->hwts + mcu2n3_detat;
        pub_m3508[3].publish(new_msg);
    }
}

void capppsCallback(const uavmotor::pulsecap::ConstPtr& pmsg)
{
    last_mcucap_valid = true;
    last_mcucap_rosts = pmsg->header.stamp;
    last_mcucap_hwts = pmsg->hwts;

    if
    (
            last_mcucap_valid == true
        &&  last_n3pps_valid == true
        &&  fabs((last_mcucap_rosts - last_n3pps_rosts).toSec()) < 0.01
    )
    {
        mcu2n3_detat = last_n3pps_hwts - last_mcucap_hwts;
        mcu2n3_detat_valid = true;
        // printf("cappps dt = %f sec\r\n", mcu2n3_detat.toSec());
    }
}

ros::Time lastcapcamera_hwts;
ros::Time lastcapcamera_rosts;
bool lastcapcamera_valid = false;


ros::Time last_synceddepth_rosts;
ros::Time last_synceddepth_hwts;
bool last_synceddepth_valid = false;

ros::Time last_syncedinfra1_rosts;
ros::Time last_syncedinfra1_hwts;
bool last_syncedinfra1_valid = false;


ros::Time last_syncedinfra2_rosts;
ros::Time last_syncedinfra2_hwts;
bool last_syncedinfra2_valid = false;



void capcameraCallback(const uavmotor::pulsecap::ConstPtr& pmsg)
{
    if(mcu2n3_detat_valid)
    {
        lastcapcamera_valid = true;
        lastcapcamera_hwts = pmsg->hwts + mcu2n3_detat;
        lastcapcamera_rosts = pmsg->header.stamp;
    }

}

void depthCallback(const sensor_msgs::Image::ConstPtr& pmsg)
{  
    sensor_msgs::Image new_msg;
    
    if(lastcapcamera_valid == true)
    {
        float dt = (pmsg->header.stamp - lastcapcamera_rosts).toSec();
        // dt = fabs(dt);
        if(dt > 0.007 && dt <= 0.016)
        {
            new_msg = *pmsg;

            new_msg.header.stamp = lastcapcamera_hwts;
            pub_depth.publish(new_msg);
            
            last_synceddepth_valid = true;
            last_synceddepth_rosts = pmsg->header.stamp;            
            last_synceddepth_hwts = lastcapcamera_hwts;            
        }
        else if(last_synceddepth_valid == true)
        {
            new_msg = *pmsg;
            new_msg.header.stamp = last_synceddepth_hwts + (pmsg->header.stamp - last_synceddepth_rosts);
            pub_depth.publish(new_msg);
        }
        // printf("depth:%f\r\n", new_msg.header.stamp.toSec());

    }
}




void infra1Callback(const sensor_msgs::Image::ConstPtr& pmsg)
{
    sensor_msgs::Image new_msg;
    if(lastcapcamera_valid == true)
    {
        float dt = (pmsg->header.stamp - lastcapcamera_rosts).toSec();
        // dt = fabs(dt);
        if(dt > 0.007 && dt <= 0.016)
        {
            new_msg = *pmsg;
            new_msg.header.stamp = lastcapcamera_hwts;

            pub_infra1.publish(new_msg);

            last_syncedinfra1_valid = true;
            last_syncedinfra1_rosts = pmsg->header.stamp;            
            last_syncedinfra1_hwts = lastcapcamera_hwts;
        }
        else if(last_syncedinfra1_valid == true)
        {
            new_msg = *pmsg;
            new_msg.header.stamp = last_syncedinfra1_hwts + (pmsg->header.stamp - last_syncedinfra1_rosts);
            pub_infra1.publish(new_msg);
        }
        // ROS_INFO("new_msg = %f   lastcapcamera_hwts = %f   pmsg->header.stamp = %f    last_syncedinfra1_rosts = %f\r\n", new_msg.header.stamp.toSec(), lastcapcamera_hwts.toSec(), pmsg->header.stamp.toSec(), last_syncedinfra1_rosts.toSec());

    }
}

void infra2Callback(const sensor_msgs::Image::ConstPtr& pmsg)
{
    sensor_msgs::Image new_msg;
    if(lastcapcamera_valid == true)
    {
        float dt = (pmsg->header.stamp - lastcapcamera_rosts).toSec();
        // dt = fabs(dt);
        if(dt > 0.007 && dt <= 0.016)
        {
            new_msg = *pmsg;
            new_msg.header.stamp = lastcapcamera_hwts;

            pub_infra2.publish(new_msg);

            last_syncedinfra2_valid = true;
            last_syncedinfra2_rosts = pmsg->header.stamp;            
            last_syncedinfra2_hwts = lastcapcamera_hwts;            
        }
        else if(last_syncedinfra2_valid == true)
        {
            new_msg = *pmsg;
            new_msg.header.stamp = last_syncedinfra2_hwts + (pmsg->header.stamp - last_syncedinfra2_rosts);
            pub_infra2.publish(new_msg);
        }
    }
}
 
 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "timeconvert");
    ros::NodeHandle n("~");

    ros::Subscriber sub_imu = n.subscribe("/djiros/imu_hwts", 2000, imuCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_n3pps = n.subscribe("/djiros/pulse", 2000, n3ppsCallback, ros::TransportHints().tcpNoDelay());
    // ros::Subscriber sub_n3gps = n.subscribe("/djiros/gps", 2000, n3gpsCallback, ros::TransportHints().tcpNoDelay());


    ros::Subscriber sub_target_rpm = n.subscribe("/m100withm3508/target_rpm", 2000, ctrlRPMCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_ctrl_current = n.subscribe("/m100withm3508/ctrl_current", 2000, ctrlCurrentCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_m3508[4];
    sub_m3508[0] = n.subscribe("/m100withm3508/m3508_m1", 2000, m3508m1Callback, ros::TransportHints().tcpNoDelay());
    sub_m3508[1] = n.subscribe("/m100withm3508/m3508_m2", 2000, m3508m2Callback, ros::TransportHints().tcpNoDelay());
    sub_m3508[2] = n.subscribe("/m100withm3508/m3508_m3", 2000, m3508m3Callback, ros::TransportHints().tcpNoDelay());
    sub_m3508[3] = n.subscribe("/m100withm3508/m3508_m4", 2000, m3508m4Callback, ros::TransportHints().tcpNoDelay());


    ros::Subscriber sub_cappps = n.subscribe("/m100withm3508/cap_n3pps", 2000, capppsCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_capcamera = n.subscribe("/m100withm3508/cap_camerapulse", 2000, capcameraCallback, ros::TransportHints().tcpNoDelay());


    ros::Subscriber sub_depth = n.subscribe("/camera/depth/image_rect_raw", 10, depthCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_infra1 = n.subscribe("/camera/infra1/image_rect_raw", 10, infra1Callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_infra2 = n.subscribe("/camera/infra2/image_rect_raw", 10, infra2Callback, ros::TransportHints().tcpNoDelay());



    pub_imu = n.advertise<sensor_msgs::Imu>("/synced/imu", 10);
    // pub_gps = n.advertise<sensor_msgs::NavSatFix>("/synced/gps", 10);
    pub_target_rpm = n.advertise<uavmotor::motordata>("/synced/target_rpm", 10);
    pub_ctrl_current = n.advertise<uavmotor::motordata>("/synced/ctrl_current", 10);

    pub_m3508[0] = n.advertise<uavmotor::m3508>("/synced/m3508_m1", 10);
    pub_m3508[1] = n.advertise<uavmotor::m3508>("/synced/m3508_m2", 10);
    pub_m3508[2] = n.advertise<uavmotor::m3508>("/synced/m3508_m3", 10);
    pub_m3508[3] = n.advertise<uavmotor::m3508>("/synced/m3508_m4", 10);

    pub_depth= n.advertise<sensor_msgs::Image>("/synced/depth", 10);
    pub_infra1= n.advertise<sensor_msgs::Image>("/synced/infra1", 10);
    pub_infra2= n.advertise<sensor_msgs::Image>("/synced/infra2", 10);
    

    ros::spin();

    return 0;
}