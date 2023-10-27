#include "gps.hpp"

std::shared_ptr<Eigen::Vector3d> GPS::Gnss2Enu(int filter_para,
                                               double latitude,
                                               double longitude,
                                               double altitude)
{
    //PCL可视化：绿点为GPS点，红点为内点
    // static pcl::visualization::CloudViewer viewer("cloud");
    // viewer.runOnVisualizationThread(std::bind(&GPS::VisualizationCallback, this, std::placeholders::_1));


    std::shared_ptr<Eigen::Vector3d> point_enu = std::make_shared<Eigen::Vector3d>();
    pcl::PointXYZ new_point;
    if (_is_first_time)
    {
        _local_cartesian.Reset(latitude, longitude, altitude);
        _is_first_time = false;
        _local_cartesian.Forward(latitude, longitude, altitude, point_enu->x(), point_enu->y(), point_enu->z());
        // _points_enu_ptr.push_back(point_enu);
        point_enu->z() = 0.0;

        new_point.x = point_enu->x();
        new_point.y = point_enu->y();
        new_point.z = point_enu->z();
        std::unique_lock<std::mutex> lockOnCallBack(_cloud_mutex);
        {
            _cloud->points.push_back(new_point);
        }
        PointsPushBack(point_enu);
        _cloud->width = _cloud->points.size();
        _cloud->height = 1; // 通常设为1，表示是无组织的点云
    }
    _local_cartesian.Forward(latitude, longitude, altitude, point_enu->x(), point_enu->y(), point_enu->z());

    if (!_points_enu_ptr.empty())
    {
        // 获取当前点和最后一个点的坐标
        Eigen::Vector3d current_point = *point_enu;
        Eigen::Vector3d last_point = *_points_enu_ptr.back();

        current_point == last_point;
        current_point.z() = 0.0;
        last_point.z() = 0.0;

        double point_x = current_point.x();
        double last_point_x = last_point.x();
        double point_y = current_point.y();
        double last_point_y = last_point.y();
        double point_z = current_point.z();
        double last_point_z = last_point.z();

        // 定义一个小的正数用于浮点数比较
        const double epsilon = 0.001;

        // 进行比较
        if (fabs(point_x - last_point_x) > epsilon ||
            fabs(point_y - last_point_y) > epsilon ||
            fabs(point_z - last_point_z) > epsilon)
        {
            point_enu->z() = 0.0;
            if (PointFilter(filter_para, *point_enu))
            {
                new_point.x = point_enu->x();
                new_point.y = point_enu->y();
                new_point.z = point_enu->z();
                {
                    std::unique_lock<std::mutex> lockOnCallBack(_cloud_mutex);
                    _cloud->points.push_back(new_point);
                }
                PointsPushBack(point_enu);
                _cloud->width = _cloud->points.size();
                _cloud->height = 1;
                // RansacPcl(bestDP, bestPointOnLine);
                // double length;
                // Eigen::Vector3d line_vector;
                // RansacAndComputeLength(10, 1000, 3, 3, length, line_vector);
            }
        }
        else
        {
        }
    }
    return point_enu;
}

void GPS::PointsPushBack(std::shared_ptr<Eigen::Vector3d> &point_enu)
{
    std::unique_lock<std::mutex> lk(_points_mutex);
    _points_enu_ptr.push_back(point_enu);
}

long int GPS::CheckPointsSize()
{
    std::unique_lock<std::mutex> lk(_points_mutex);
    long int num;
    num = _points_enu_ptr.size();
    return num;
}

Eigen::Vector3d GPS::GetPoints(long int index)
{
    std::unique_lock<std::mutex> lk(_points_mutex);
    Eigen::Vector3d tmp;
    tmp = *_points_enu_ptr[index];
    return tmp;
}

void GPS::VisualizationCallback(pcl::visualization::PCLVisualizer &viz)
{
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(_cloud, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(_cloud, 255, 0, 0);
    std::unique_lock<std::mutex> lockOnCallBack(_cloud_mutex);
    {
        if (!viz.updatePointCloud(_cloud, green, "cloud1"))
        {
            viz.addPointCloud(_cloud, green, "cloud1");
        }

        if (!viz.updatePointCloud(_cloud_line, red, "cloud2"))
        {
            viz.addPointCloud(_cloud_line, red, "cloud2");
        }

        // 设置点的大小
        viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud1");
        viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud2");
    }
}

bool GPS::RansacPcl(int dis_threshold, int max_iterations_time)
{
    // Eigen::Vector3d bestDP;
    // Eigen::Vector3d bestPointOnLine;
    // static long int lastn_pcl = 0;
    // long int n;
    // n = _cloud->size();
    // // std::cout << "点数量 : " << n << std::endl;
    // if (n - lastn_pcl < 10) // 每加入10个点就进行ransac
    // {
    //     return false;
    // }
    // lastn_pcl = n;

    // cout << "点云点数为：" << _cloud->points.size() << endl;
    //-----------------------------拟合直线-----------------------------
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_line(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(_cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_line);
    ransac.setDistanceThreshold(dis_threshold);   // 内点到模型的最大距离
    ransac.setMaxIterations(max_iterations_time); // 最大迭代次数
    ransac.computeModel();                        // 直线拟合
    //--------------------------根据索引提取内点------------------------
    std::vector<int> inliers;
    ransac.getInliers(inliers);

    std::unique_lock<std::mutex> lockOnCallBack(_cloud_mutex);
    {
        pcl::copyPointCloud<pcl::PointXYZ>(*_cloud, inliers, *_cloud_line);
    }

    //----------------------------输出模型参数--------------------------
    Eigen::VectorXf coef;
    ransac.getModelCoefficients(coef);
    _bestPointOnLine << coef[0], coef[1], coef[2];
    _bestDP << coef[3], coef[4], coef[5];
    double length;
    return true;
    // ComputeLength(3, 3, length);
}

bool GPS::PointFilter(int window_amount, Eigen::Vector3d point_enu)
{
    double best_dis = 0;
    if (_points_enu_ptr.size() >= window_amount)
    {
        Eigen::Vector3d sum(0, 0, 0); // 用于存储和的向量

        // 使用迭代器取出最后五个元素并求和
        for (auto it = _points_enu_ptr.end() - window_amount; it != _points_enu_ptr.end(); ++it)
        {
            sum += **it;
        }
        Eigen::Vector3d mean = sum / window_amount;
        for (auto it = _points_enu_ptr.end() - window_amount; it != _points_enu_ptr.end(); ++it)
        {
            double distance_squared = pow((*it)->x() - mean.x(), 2) + pow((*it)->y() - mean.y(), 2); // 只包括x和y
            if (distance_squared > best_dis)
            {
                best_dis = distance_squared;
            }
        }

        if ((pow(point_enu.x() - mean.x(), 2) + pow(point_enu.y() - mean.y(), 2)) > best_dis)
            return true;
        else
            return false;
    }
    else
    {
        return true;
        std::cout << "向量中的元素不足5个。" << std::endl;
    }
    return true;
}

bool GPS::ComputeLength(double &length, int avg_dis_amount)
{
    // 逻辑3：所有内点中找距离最远的两个点
    std::priority_queue<double, std::vector<double>, std::greater<double>> min_heap;

    // 遍历所有内点点云，计算每一对点在line上的投影点之间的距离
    for (int i = 0; i < _cloud_line->points.size(); ++i)
    {
        for (int j = i + 1; j < _cloud_line->points.size(); ++j)
        {
            Eigen::Vector3d a(_cloud_line->points[i].x, _cloud_line->points[i].y, _cloud_line->points[i].z);
            Eigen::Vector3d b(_cloud_line->points[j].x, _cloud_line->points[j].y, _cloud_line->points[j].z);

            Eigen::Vector3d vector_to_a = a - _bestPointOnLine;
            Eigen::Vector3d vector_to_b = b - _bestPointOnLine;
            Eigen::Vector3d normalized_line_direction = _bestDP.normalized(); // 归一化
            double t_a = vector_to_a.dot(normalized_line_direction);
            double t_b = vector_to_b.dot(normalized_line_direction);

            // 计算两个投影长度的差值
            double distance = std::abs(t_a - t_b);

            // 更新最大堆
            if (min_heap.size() < avg_dis_amount)
            {
                min_heap.push(distance);
            }
            else if (distance > min_heap.top())
            {
                min_heap.pop();
                min_heap.push(distance);
            }
        }
    }

    // 计算最远的前avg_dis_amount个距离的平均值
    double sum = 0;
    int count = 0;
    while (!min_heap.empty())
    {
        sum += min_heap.top();
        std::cout << "top max dis: " << min_heap.top() << std::endl;
        min_heap.pop();
        count++;
    }
    length = sum / count;

    std::cout << "Average of max distances: " << length << std::endl;

    return true;

    // 逻辑2：距离 = 内点最前的几个点 与 点云最远的距离的平均
    // 计算一开始的front_lnliers_amount个点的平均坐标
    // Eigen::Vector3d avg_first(0, 0, 0);
    // for (int i = 0; i < front_lnliers_amount; ++i)
    // {
    //     pcl::PointXYZ point = _cloud_line->points[i];
    //     avg_first += Eigen::Vector3d(point.x, point.y, point.z);
    // }
    // avg_first /= front_lnliers_amount;

    // // 初始化一个最小堆
    // std::priority_queue<double, std::vector<double>, std::greater<double>> min_heap;

    // // 预计算点云大小和遍历范围
    // size_t cloud_size = _cloud_line->points.size();
    // int end_idx = cloud_size - front_lnliers_amount;

    // // 创建一个Vector3d对象用于存储当前点
    // Eigen::Vector3d current_point;

    // // 遍历所有内点点云
    // for (int i = front_lnliers_amount; i < end_idx; ++i)
    // {
    //     pcl::PointXYZ point = _cloud_line->points[i];
    //     current_point = Eigen::Vector3d(point.x, point.y, point.z);

    //     // 计算当前点到avg_first的距离
    //     double current_distance = (current_point - avg_first).norm();

    //     // 更新最小堆
    //     if (min_heap.size() < back_lnliers_amount)
    //     {
    //         min_heap.push(current_distance);
    //     }
    //     else if (current_distance > min_heap.top())
    //     {
    //         min_heap.pop();
    //         min_heap.push(current_distance);
    //     }
    // }

    // // 计算最远的前back_lnliers_amount个距离的平均值
    // double sum = 0;
    // int count = 0;
    // while (!min_heap.empty())
    // {
    //     sum += min_heap.top();
    //     min_heap.pop();
    //     count++;
    // }
    // length = sum / count;

    // std::cout << "length : " << length << std::endl;
    // std::cout << "cloud size : " <<cloud_size <<std::endl;
}

int GPS::GetCloudSize()
{
    return _cloud->points.size();
}

bool GPS::RansacAndComputeLength(double &length,
                                 Eigen::Vector3d &line_vector,
                                 Eigen::Vector3d &point_on_line,
                                 int dis_threshold,
                                 int max_iteration_time,
                                 int avg_dis_amount)
{
    if (RansacPcl(dis_threshold, max_iteration_time))
    {
        ComputeLength(length, avg_dis_amount);
        line_vector = _bestDP;
        std::cout << "line vector : \n"
                  << line_vector << std::endl;
        return true;
    }
    return false;
}

bool GPS::ConvertAndSavePoint(double latitude,
                              double longitude,
                              double altitude,
                              int filter_para)
{

    Gnss2Enu(filter_para, latitude, longitude, altitude);
    return true;
}