#ifndef _GPS_
#define _GPS_

#include <functional>
#include "rclcpp/rclcpp.hpp"

// opencv

// ros message
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <string>

#include <GeographicLib/LocalCartesian.hpp>
#include <Eigen/Core>
#include <vector>
#include <algorithm>
#include <random>
#include <iostream>
#include <thread>
#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iomanip>
#include <queue>

class GPS
{
public:
    GPS()
    {
        _is_first_time = true;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        _cloud = cloud;
        _cloud_line = cloud2;
    }

    /**
     * @brief 将GNSS坐标转换为ENU坐标，并进行点云可视化。
     *
     * @param [in] filter_para 过滤窗口大小。
     * @param [in] latitude 纬度。
     * @param [in] longitude 经度。
     * @param [in] altitude 高度。
     * @return [out] 返回转换后的ENU坐标。
     */
    std::shared_ptr<Eigen::Vector3d> Gnss2Enu(int filter_para,
                                              double latitude,
                                              double longitude,
                                              double altitude);

    void PointsPushBack(std::shared_ptr<Eigen::Vector3d> &point_enu);
    long int CheckPointsSize();
    Eigen::Vector3d GetPoints(long int index);
    bool RansacPcl(int dis_threshold, int max_iterations_time);
    void VisualizationCallback(pcl::visualization::PCLVisualizer &viz);
    bool PointFilter(int window_amount, Eigen::Vector3d point_enu);
    bool ComputeLength(double &length, int avg_dis_amount);
    int GetCloudSize();

    /**
     * @brief 转换并保存点。
     *
     * @param [in] latitude 纬度。
     * @param [in] longitude 经度。
     * @param [in] altitude 高度。
     * @param [in] filter_para 过滤窗口大小。
     * @return [out] 如果转换和保存成功，返回true，否则返回false。
     */
    bool ConvertAndSavePoint(double latitude,
                             double longitude,
                             double altitude,
                             int filter_para = 8);
    /**
     * @brief 使用RANSAC算法拟合直线并计算长度。
     *
     * @param [out] length 输出的长度。
     * @param [out] line_vector 输出的直线向量。
     * @param [out] point_on_line 输出的直线上的点。
     * @param [in] dis_threshold ransac距离阈值。
     * @param [in] max_iteration_time 最大迭代次数。
     * @param [in] avg_dis_amount 计算平均最长距离的样本数量。
     * @return [out] 如果拟合和计算成功，返回true，否则返回false。
     */
    bool RansacAndComputeLength(double &length,
                                Eigen::Vector3d &line_vector,
                                Eigen::Vector3d &point_on_line,
                                int dis_threshold = 5,
                                int max_iteration_time = 1000,
                                int avg_dis_amount = 3);

private:
    GeographicLib::LocalCartesian _local_cartesian;
    bool _is_first_time;
    std::vector<std::shared_ptr<Eigen::Vector3d>> _points_enu_ptr;
    std::mutex _points_mutex;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_line;
    std::mutex _cloud_mutex;
    Eigen::Vector3d _bestDP;
    Eigen::Vector3d _bestPointOnLine;
};

#endif