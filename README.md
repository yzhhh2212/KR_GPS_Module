## 1. 功能描述
* 将GNSS坐标系转换为ENU坐标系
* 对一系列转换后的ENU坐标点拟合成为直线

## 2. 目录结构
```shell

├── CMakeLists.txt            
├── include
│   └── gps
│       ├── gps.hpp         #头文件
│       └── gps_sub.hpp     #基于ros2的demo
├── package.xml
└── src
    ├── gps.cpp             #对GPS功能的实现
    └── gps_sub.cpp         #demo的main函数

```

## 3. 工程依赖库
运行此功能包提前安装以下依赖库:
* PCL
* Eigen
* ROS2
* GeographicLib

## 4. 使用方法
* 输入：
```c++
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
```

* 输出：
```c++
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
```

* demo使用：
在gps.cpp文件中将第9,10行代码解注释，即可看到可视化的GPS点云
```c++
#include "gps.hpp"

std::shared_ptr<Eigen::Vector3d> GPS::Gnss2Enu(int filter_para,
                                               double latitude,
                                               double longitude,
                                               double altitude)
{
    //PCL可视化：绿点为GPS点，红点为内点
    static pcl::visualization::CloudViewer viewer("cloud");
    viewer.runOnVisualizationThread(std::bind(&GPS::VisualizationCallback, this, std::placeholders::_1));
    ...
```