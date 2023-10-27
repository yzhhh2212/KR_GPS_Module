#ifndef _GPS_SUB_
#define _GPS_SUB_

#include <functional>
#include "rclcpp/rclcpp.hpp"
#include <memory>

// opencv

// ros message
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <string>
#include <thread>
#include <string>
#include <fstream>
#include <sstream>
#include <queue>
#include <spdlog/spdlog.h>

class GPS_SUB : public rclcpp::Node
{

public:
    GPS_SUB(
        const std::string &name,
        const std::string &topic_gps,
        const std::string &topic_pub,
        std::function<std::shared_ptr<Eigen::Vector3d>(int filter_para, double latitude, double longitude, double altitude)> func)
        : Node(name), _process(func)
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        _gps_sub = create_subscription<sensor_msgs::msg::NavSatFix>(
            topic_gps,
            qos,
            std::bind(&GPS_SUB::Callback, this, std::placeholders::_1));
        _marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(topic_pub, qos);
        _path_pub = this->create_publisher<nav_msgs::msg::Path>("path", qos);
        _path_msg.header.frame_id = "map";
        _gps_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps_txt", 10);
        // _sender_thread = std::thread(&GPS_SUB::Send_Data, this);
        _line_publisher = this->create_publisher<visualization_msgs::msg::Marker>("ransac_line", 10);
        _gps_instance = std::make_shared<GPS>();
    }

    ~GPS_SUB()
    {
        // _sender_thread.join();
    }

private:
    /**
     * @brief 使用示例DEMO
     */
    void Callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {

        // demo:
        double altitude = msg->altitude;
        double longitude = msg->longitude;
        double latitude = msg->latitude;
        double length;
        static long int lastn_pcl = 0;
        long int n;
        Eigen::Vector3d line_vector;
        Eigen::Vector3d point_on_line;
        std::shared_ptr<Eigen::Vector3d> point_enu;
        int filter_para = 5;

        _gps_instance->ConvertAndSavePoint(latitude, longitude, altitude); // GNSS点入口，负责转化成ENU点并且保存
        n = _gps_instance->GetCloudSize();
        if (n - lastn_pcl > 10) // 每加入10个点就进行ransac
        {
            lastn_pcl = n;
            _gps_instance->RansacAndComputeLength(length, line_vector, point_on_line); // 出口：输出拟合直线长度,直线方向向量,以及直线上的某个点
        }

        // DrawRansacLine();
        // DrawEnuPoint(point_enu, msg);
        // DrawPath(point_enu, msg);
    }

    bool Send_Data()
    {
        std::cout << "进入gps发送" << std::endl;
        std::string gps_file = "/usr/local/project/keystar/GPS/gps2.txt";
        std::ifstream if_gps(gps_file);
        if (!if_gps)
        {
            spdlog::error("GPS data file open failed. {}", gps_file);
            return false;
        }
        std::string line;
        std::cout << "等待3秒" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        std::cout << "开始发送" << std::endl;
        std::string item;
        while (std::getline(if_gps, line))
        {
            sensor_msgs::msg::NavSatFix gps_msg;
            std::istringstream iss(line);
            std::getline(iss, item, ',');
            double ts = std::stod(item);

            std::getline(iss, item, ',');
            double lat = std::stod(item);

            std::getline(iss, item, ',');
            double lon = std::stod(item);

            std::getline(iss, item, ',');
            double alt = std::stod(item);
            gps_msg.header.stamp.sec = static_cast<int32_t>(ts);
            gps_msg.latitude = lat;
            gps_msg.longitude = lon;
            gps_msg.altitude = alt;
            _gps_pub->publish(gps_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if_gps.close();
        return true;
    }

    bool DrawRansacLine()
    {

        // 初始化Marker消息
        visualization_msgs::msg::Marker line;
        line.header.frame_id = "map"; // 参考框架
        line.header.stamp = this->now();
        line.ns = "lines";
        line.action = visualization_msgs::msg::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.id = 0;
        line.type = visualization_msgs::msg::Marker::LINE_LIST;

        // 设置线的宽度和颜色
        line.scale.x = 0.1;
        line.color.a = 1.0;
        line.color.r = 1.0;
        line.color.g = 0.0;
        line.color.b = 0.0;

        // 计算两个点
        geometry_msgs::msg::Point p1, p2;
        p1.x = _bestPointONLine[0];
        p1.y = _bestPointONLine[1];
        p1.z = _bestPointONLine[2];

        // 假设 t = 10.0（延伸的长度）
        Eigen::Vector3d end_point = _bestPointONLine + 40.0 * _bestDp;

        p2.x = end_point[0];
        p2.y = end_point[1];
        p2.z = end_point[2];

        // 添加到点列表
        line.points.push_back(p1);
        line.points.push_back(p2);

        // 发布Marker
        _line_publisher->publish(line);
    }

    void DrawEnuPoint(std::shared_ptr<Eigen::Vector3d> &point_enu, const sensor_msgs::msg::NavSatFix::SharedPtr &msg)
    {

        auto marker_msg = visualization_msgs::msg::Marker();
        marker_msg.lifetime = rclcpp::Duration(100000, 0);
        marker_msg.header.frame_id = "map";
        marker_msg.header.stamp = msg->header.stamp;
        marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
        marker_msg.action = visualization_msgs::msg::Marker::ADD;
        marker_msg.pose.position.x = point_enu->x();
        marker_msg.pose.position.y = point_enu->y();
        // marker_msg.pose.position.z = point_enu->z();
        marker_msg.pose.position.z = 0.0;
        // std::cout << " x :  " << point_enu->x()
        //           << " y :  " << point_enu->y()
        //           << " z :  " << point_enu->z() << std::endl;
        // std::cout << "---------------------------------------" << std::endl;

        marker_msg.scale.x = 1.0;
        marker_msg.scale.y = 1.0;
        marker_msg.scale.z = 1.0;
        marker_msg.color.r = 1.0;
        marker_msg.color.g = 0.0;
        marker_msg.color.b = 0.0;
        marker_msg.color.a = 1.0;
        _marker_pub->publish(marker_msg);
    }

    void DrawPath(std::shared_ptr<Eigen::Vector3d> &point_enu, const sensor_msgs::msg::NavSatFix::SharedPtr &msg)
    {

        geometry_msgs::msg::PoseStamped new_pose;
        new_pose.pose.position.x = point_enu->x();
        new_pose.pose.position.y = point_enu->y();
        // new_pose.pose.position.z = point_enu->z();
        new_pose.pose.position.z = 0.0;
        new_pose.header.frame_id = "map";
        new_pose.header.stamp = msg->header.stamp;

        // 将新的PoseStamped添加到Path消息的poses数组中
        _path_msg.poses.push_back(new_pose);

        // 更新Path消息的头部时间戳
        _path_msg.header.stamp = msg->header.stamp;

        // 发布Path消息
        _path_pub->publish(_path_msg);
    }

private:
    // 处理用户的回调函数
    std::function<std::shared_ptr<Eigen::Vector3d>(int filter_para, double latitude, double longitude, double altitude)> _process;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _gps_sub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _marker_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _line_publisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _path_pub;
    nav_msgs::msg::Path _path_msg;
    std::thread _sender_thread;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr _gps_pub;
    Eigen::Vector3d _bestDp;
    Eigen::Vector3d _bestPointONLine;
    std::shared_ptr<GPS> _gps_instance;
};

#endif