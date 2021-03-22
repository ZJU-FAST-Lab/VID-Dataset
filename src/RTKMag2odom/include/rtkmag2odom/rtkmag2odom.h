#ifndef RTKMAG2ODOM_H
#define RTKMAG2ODOM_H

#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <std_msgs/Float64MultiArray.h>



namespace rtkmag2odom
{
    const double PI = 3.141592653589793;
    const double TAU = 6.283185307179587;
    
    struct Config
    {
        // Subscribed Topics
        std::string RTKTopic;
        std::string MagTopic;
        // Published Topics
        std::string OdomTopic;
        
        int mode;
        double yaw_north;
        std::vector<double> origin_ground;
        // double theta_NED2Ground;
        double yaw_ground;
    };

    typedef struct
    {
        double longitude;               // unit: deg
        double latitude;                // unit: deg
        double altitude;                // unit: m
    }GpsDataType;

    typedef struct
    {
        double x_north;                 // unit: m
        double y_east;                  // unit: m
        double z_down;                  // unit: m
    }NedDataType;
    typedef struct
    {
        double x_east;                 // unit: m
        double y_north;                  // unit: m
        double z_upper;                  // unit: m
    }ENUDataType;

    typedef struct
    {
        double x;                 // unit: m
        double y;                  // unit: m
        double z;                  // unit: m
    }LocalCartesianDataType;

    //utility
    double quaternion2yaw(geometry_msgs::Quaternion q);
    geometry_msgs::Quaternion yaw2quaternion(double yaw);

    class Transformer
    {
    public:
        Transformer(Config & conf, ros::NodeHandle & nh_);
        ~Transformer(){};
        
        void RTKcallback_mode0(const sensor_msgs::NavSatFix::ConstPtr &msg);
        void Magcallback_mode0(const std_msgs::Float64MultiArray::ConstPtr &msg);
        void RTKcallback_mode1(const sensor_msgs::NavSatFix::ConstPtr &msg);
        void Magcallback_mode1(const std_msgs::Float64MultiArray::ConstPtr &msg);
        void RTKcallback_mode2(const sensor_msgs::NavSatFix::ConstPtr &msg);
        void Magcallback_mode2(const std_msgs::Float64MultiArray::ConstPtr &msg);

        void NED2FLU(NedDataType ned, LocalCartesianDataType &point_local, int mode);

        void ENU2FLU(ENUDataType enu, LocalCartesianDataType &point_local, int mode);
        
        void run();
        // GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
    private:
        
        Config config;
        
        ros::NodeHandle nh;
        ros::Subscriber RTKSub;
        ros::Subscriber MagSub;
        ros::Publisher odomPub;

        GeographicLib::LocalCartesian RTK2NED;
        GeographicLib::LocalCartesian RTK2ENU;


        double latitude;
        double longitude;
        double altitude;

        GpsDataType point_rtk;
        NedDataType point_ned;
        ENUDataType point_enu;
        LocalCartesianDataType point_odom;  //mode 0
        LocalCartesianDataType point_ground;  //mode 1

        nav_msgs::Odometry Odom;
        geometry_msgs::PoseStamped Odom_mocap;
        geometry_msgs::Quaternion orientation;
        double yaw;

        // mode 0
        bool get_init_orientation = false;
        bool begin_position = false;
        bool get_init_position = false;
        double yaw0;
        GpsDataType point_rtk0;

    };

}



#endif