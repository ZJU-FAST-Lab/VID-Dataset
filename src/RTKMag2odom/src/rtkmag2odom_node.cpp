#include <rtkmag2odom/rtkmag2odom.h>

using namespace std;
using namespace rtkmag2odom;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "rtkmag2odom_node");
    ros::NodeHandle nh_;

    Config conf;
    ros::NodeHandle nh_priv("~");

    nh_priv.getParam("RTKTopic", conf.RTKTopic);
    nh_priv.getParam("MagTopic", conf.MagTopic);
    nh_priv.getParam("OdomTopic", conf.OdomTopic);
    nh_priv.getParam("mode", conf.mode);
    nh_priv.getParam("yaw_north", conf.yaw_north);
    // nh_priv.getParam("theta_NED2Ground", conf.theta_NED2Ground);
    nh_priv.getParam("origin_ground", conf.origin_ground);
    nh_priv.getParam("yaw_ground", conf.yaw_ground);

    // cout << "ground: " << conf.RTKTopic << endl;
    // cout << "ground: " << conf.MagTopic << endl;
    // cout << "ground: " << conf.OdomTopic << endl;
    // cout << "ground: " << conf.mode << endl;
    // cout << "ground: " << conf.yaw_north << endl;
    // cout << "ground: " << conf.yaw_ground << endl;
    // cout << "ground: " << conf.origin_ground.at(0) << endl;

    // Transformer transformer(conf, nh_);

    Transformer *transformer = new Transformer(conf, nh_);
    
    while(ros::ok()) //watch dog
    {
        if(transformer == nullptr)
            transformer = new Transformer(conf, nh_);
        
        transformer->run();
        
        if(transformer)
        {
            delete transformer;
            transformer = nullptr;
        }
    }

    return 0;
}
