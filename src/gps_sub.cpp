#include "gps.hpp"
#include "gps_sub.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    GPS gps_instance;
    auto bound_function = std::bind(&GPS::Gnss2Enu, &gps_instance, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,std::placeholders::_4);
    GPS_SUB::SharedPtr sub(new GPS_SUB("GPS2ENU", "gps", "enu", bound_function));
    
    // std::thread ransac = std::thread(&GPS::RansacFitLineoThread,&gps_instance);
    
    rclcpp::spin(sub);
    rclcpp::shutdown();
    // ransac.join();

    return 0;
}