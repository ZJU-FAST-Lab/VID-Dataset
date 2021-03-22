#include <rtk_zhd_parser/rtk_zhd_parser.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rtk_zhd_parser_node");
    ros::NodeHandle n;
    RTK_ZHD_Parser rtkParser(n);
    rtkParser.runParser();
    return 0;
}
