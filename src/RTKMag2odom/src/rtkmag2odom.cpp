#include <rtkmag2odom/rtkmag2odom.h>

using namespace std;
using namespace ros;
using namespace Eigen;
using namespace GeographicLib;

GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());

namespace rtkmag2odom
{
    Transformer::Transformer(Config& conf, NodeHandle & nh_)
    :config(conf), nh(nh_)
    {
        if(config.mode == 0)  //odom mode
        {
            RTKSub = nh.subscribe(config.RTKTopic, 10, &Transformer::RTKcallback_mode0, this, ros::TransportHints().tcpNoDelay());
            MagSub = nh.subscribe(config.MagTopic, 10, &Transformer::Magcallback_mode0, this, ros::TransportHints().tcpNoDelay());

            odomPub = nh.advertise<nav_msgs::Odometry>(config.OdomTopic, 1);     
        }
        else if(config.mode == 1)  //mocap mode: out put format is odom mode and nav_msgs::Odometry not <geometry_msgs::PoseStamped>
        {
            RTKSub = nh.subscribe(config.RTKTopic, 10, &Transformer::RTKcallback_mode1, this, ros::TransportHints().tcpNoDelay());
            MagSub = nh.subscribe(config.MagTopic, 10, &Transformer::Magcallback_mode1, this, ros::TransportHints().tcpNoDelay());

            RTK2ENU = GeographicLib::LocalCartesian(config.origin_ground.at(1), config.origin_ground.at(0), config.origin_ground.at(2), earth);  //ENU coordinate 

            // geometry_msgs::Quaternion aaa  =  yaw2quaternion(1.57);
            // cout << "yaw: " << endl << aaa.w << endl << aaa.x << endl << aaa.y << endl << aaa.z << endl;
            // odomPub = nh.advertise<geometry_msgs::PoseStamped>(config.OdomTopic, 1); 
            odomPub = nh.advertise<nav_msgs::Odometry>(config.OdomTopic, 1);     

        }
        else if(config.mode == 2)  //preparing mode
        {
            RTKSub = nh.subscribe(config.RTKTopic, 10, &Transformer::RTKcallback_mode2, this, ros::TransportHints().tcpNoDelay());
            MagSub = nh.subscribe(config.MagTopic, 10, &Transformer::Magcallback_mode2, this, ros::TransportHints().tcpNoDelay());
        }
        else
        {
            ROS_ERROR("Mode error!\n");
        }
        
           
    }

    //mode 0 odom
    void Transformer::RTKcallback_mode0(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        static int cnt = 0;
        static double lat_temp=0, long_temp=0, alt_temp=0;
        if(begin_position && get_init_position)
        {
            latitude = msg->latitude;
            longitude = msg->longitude;
            altitude = msg->altitude;
            RTK2ENU.Forward(latitude, longitude, altitude, point_enu.x_east, point_enu.y_north, point_enu.z_upper);
            ENU2FLU(point_enu, point_odom, config.mode);

            Odom.header.stamp = msg->header.stamp;
            Odom.header.frame_id = "world";
            Odom.pose.pose.position.x = point_odom.x;
            Odom.pose.pose.position.y = point_odom.y;
            Odom.pose.pose.position.z = 0;
            Odom.pose.pose.orientation = yaw2quaternion(yaw);
            odomPub.publish(Odom);
        }
        else if(begin_position && !get_init_position)
        {
            long_temp += msg->longitude;
            lat_temp += msg->latitude;
            alt_temp += msg->altitude;

            cnt++;
            if(cnt > 1)
            {
                get_init_position = true;
                point_rtk0.longitude = long_temp/cnt;
                point_rtk0.latitude = lat_temp/cnt;
                point_rtk0.altitude = alt_temp/cnt;

                RTK2ENU = GeographicLib::LocalCartesian(point_rtk0.latitude, point_rtk0.longitude, point_rtk0.altitude, earth);  //ENU coordinate 

                ROS_INFO("==========Init position ok!===========\n");
            }
        }
        else 
        {
            ROS_INFO("Waiting for orientation init...\n");
        }
    }
    
    void Transformer::Magcallback_mode0(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        static int cnt = 0;
        static double yaw_temp=0;
        if(get_init_orientation) 
        {
            begin_position = true;
            // orientation = msg->orientation;
            // yaw = quaternion2yaw(orientation) - yaw0;
            yaw = msg->data[0];
        }
        else
        {
            // yaw0 = quaternion2yaw(msg->orientation);
            yaw0 = msg->data[0];
            yaw_temp += yaw0;

            cnt++;
            if(cnt > 1)  
            {
                get_init_orientation = true;
                yaw0 = yaw_temp/cnt;
                ROS_INFO("==========Init yaw ok!===========\n");
            } 
        }
    }

    //mode 1 mocap
    std::vector<NedDataType> point_ned_set;
    NedDataType point_ned_last;
    NedDataType point_ned_last2;
    void Transformer::RTKcallback_mode1(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        latitude = msg->latitude;
        longitude = msg->longitude;
        altitude = msg->altitude;
        // ROS_INFO("lat: %.9lf,\nlong: %.9lf,\nalt: %.9lf\n",latitude,longitude,altitude);
        RTK2ENU.Forward(latitude, longitude, altitude, point_enu.x_east, point_enu.y_north, point_enu.z_upper);
        ENU2FLU(point_enu, point_ground, config.mode);

        Odom.header.stamp = msg->header.stamp;
        Odom.header.frame_id = "world";
        //for enu world
        // Odom.pose.pose.position.x = point_enu.x_east;
        // Odom.pose.pose.position.y = point_enu.y_north;
        // Odom.pose.pose.position.z = 0; //point_enu.z_upper;
        //for local
        Odom.pose.pose.position.x = point_ground.x;
        Odom.pose.pose.position.y = point_ground.y;
        Odom.pose.pose.position.z = 0; 
        Odom.pose.pose.orientation = yaw2quaternion(yaw);
        odomPub.publish(Odom);

        // Odom_mocap.header.stamp = msg->header.stamp;
        // Odom_mocap.header.frame_id = "world";
        // //for enu world
        // // Odom_mocap.pose.position.x = point_enu.x_east;
        // // Odom_mocap.pose.position.y = point_enu.y_north;
        // // Odom_mocap.pose.position.z = 0; //point_enu.z_upper;
        // //for local
        // Odom_mocap.pose.position.x = point_ground.x;
        // Odom_mocap.pose.position.y = point_ground.y;
        // Odom_mocap.pose.position.z = 0; 
        // Odom_mocap.pose.orientation = yaw2quaternion(yaw);
        // odomPub.publish(Odom_mocap);
    }
    
    void Transformer::Magcallback_mode1(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        // orientation = msg->orientation;
        //for enu world
        // yaw = quaternion2yaw(orientation) + PI/2;
        // cout << "yaw: " << quaternion2yaw(orientation) << endl;
        //for local
        // yaw = quaternion2yaw(orientation) - config.yaw_ground;
        yaw = msg->data[0];
    }

    //mode 2 preparing
    void Transformer::RTKcallback_mode2(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        static int cnt = 0;
        static double lat_temp=0, long_temp=0, alt_temp=0;
        long_temp += msg->longitude;
        lat_temp += msg->latitude;
        alt_temp += msg->altitude;

        cnt++;
        if(cnt > 1)
        {
            point_rtk0.longitude = long_temp/cnt;
            point_rtk0.latitude = lat_temp/cnt;
            point_rtk0.altitude = alt_temp/cnt;
            long_temp = 0;
            lat_temp = 0;
            alt_temp = 0;
            cnt = 0;

            ROS_INFO("now rtk msg is: \nlongitude: %.9lf \n latitude: %.9lf \n altitude: \n %.9lf ;\n", point_rtk0.longitude, point_rtk0.latitude, point_rtk0.altitude);
            }
    }
    
    void Transformer::Magcallback_mode2(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        static int cnt = 0;
        static double yaw_temp=0;
        // yaw_temp += quaternion2yaw(msg->orientation);
        yaw_temp += msg->data[0];

        cnt++;
        if(cnt > 100)  
        {
            yaw0 = yaw_temp/cnt;
            yaw_temp = 0;
            cnt = 0;
            ROS_INFO("yaw now is %.9lf \n", yaw0);
        } 
    }

    
    //from NED to FLU  //mode 0 odom  mode 1 mocap
    //Front x --- Left y --- Upper z
    // form ENU to FLU
    void Transformer::NED2FLU(NedDataType ned, LocalCartesianDataType &point_local, int mode)
    {
        if(mode == 0)   //odom
        {
            double theta = yaw0 - config.yaw_north;   // TODO:ensure which direction of angle is incresing.
            double st = sin(theta);
            double ct = cos(theta);
            point_local.x = ct * ned.x_north + st * ned.y_east;
            point_local.y = st * ned.x_north - ct * ned.y_east;
            point_local.z = -ned.z_down;
            // ct,  st,  0,
            // st, -ct,  0,
            //  0,   0, -1;
        }
        else if(mode == 1)  //mocap
        {
            double theta = config.yaw_ground - config.yaw_north;   // TODO:ensure which direction of angle is incresing.
            double st = sin(theta);
            double ct = cos(theta);
            point_local.x = ct * ned.x_north + st * ned.y_east;
            point_local.y = st * ned.x_north - ct * ned.y_east;
            point_local.z = -ned.z_down;
        }
      
    }

    //Front x --- Left y --- Upper z
    // form ENU to FLU  //mode 0 odom  mode 1 mocap
    void Transformer::ENU2FLU(ENUDataType enu, LocalCartesianDataType &point_local, int mode)
    {
        //ensure which is the rotation angle
        if(mode == 0)   //odom
        {
            double theta = yaw0 - config.yaw_north + PI/2;   
            double st = sin(theta);
            double ct = cos(theta);
            point_local.x =  ct * enu.x_east + st * enu.y_north;
            point_local.y = -st * enu.x_east + ct * enu.y_north;
            point_local.z = enu.z_upper;
            // ct,  st,  0,
            //-st,  ct,  0,
            //  0,   0,  1;
        }
        else if(mode == 1)  //mocap
        {
            double theta = config.yaw_ground - config.yaw_north + PI/2;   
            //mag: xyz is NED, north is yaw=0, Counterclockwise(looking down) positive，and clockwise is negative
            //gps rtk: xyz is ENU 
            //It is different in axis, and the bias is PI/2
            double st = sin(theta);
            double ct = cos(theta);
            point_local.x =  ct * enu.x_east + st * enu.y_north;
            point_local.y = -st * enu.x_east + ct * enu.y_north;
            point_local.z = enu.z_upper;
            // ct,  st,  0,
            //-st,  ct,  0,
            //  0,   0,  1;
        }
      
    }
  
    void Transformer::run()
    {
        ros::spin();
    }

    double quaternion2yaw(geometry_msgs::Quaternion q)
    {
        double a = q.w, b = q.x, c = q.y, d = q.z;
        double y = atan2(2*(b*c+a*d), (a*a + b*b - c*c - d*d));
        return y;
    }
    geometry_msgs::Quaternion yaw2quaternion(double yaw)
    {
        double cy = cos(yaw/2);
        double sy = sin(yaw/2);
        geometry_msgs::Quaternion q;
        q.w = cy;
        q.x = 0;
        q.y = 0;
        q.z = sy;
        return q; 
    }

}

