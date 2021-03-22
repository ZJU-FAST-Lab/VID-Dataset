
#include <ros/ros.h>  
#include <librealsense2/rs.hpp>


int intercam_sync_mode = 0;

void node_param_init(ros::NodeHandle &n)
{
    n.getParam("intercam_sync_mode", intercam_sync_mode);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "syncmode");
    ros::NodeHandle n("~");
    node_param_init(n);

    rs2::context ctx;

    rs2::device_list devices_list = ctx.query_devices();

    size_t device_count = devices_list.size();
    if (!device_count)
    {
        std::cout << "No device detected. Is it plugged in?\n";
        return EXIT_SUCCESS;
    }

    rs2::device device;
    bool device_found = false;
    for (auto&& dev : devices_list)
    {
        if (dev.supports(RS2_CAMERA_INFO_PRODUCT_LINE) && 
            std::string(dev.get_info(RS2_CAMERA_INFO_PRODUCT_LINE)) == "D400")
        {
            device = dev;
            device_found = true;
            break;
        }
    }

    if (!device_found)
    {
        std::cout << "No device from D400 product line detected. Is it plugged in?\n";
        return EXIT_SUCCESS;
    }

    rs2::depth_sensor depth_sensor = device.query_sensors().front();
    depth_sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, intercam_sync_mode);

    while(ros::ok())
    {

        break;
    }
    

    return 0;
}











