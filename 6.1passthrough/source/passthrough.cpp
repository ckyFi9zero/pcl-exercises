#include <iostream>                          // 引入标准输入输出流
#include <ctime>                             // 引入时间函数，用于随机种子
#include <pcl/point_types.h>                 // 引入PCL点类型定义
#include <pcl/filters/passthrough.h>         // 引入PCL直通滤波器头文件

int main (int argc, char** argv)
{
    srand(time(0));                          // 用当前时间初始化随机数种子

    // 创建原始点云指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // 创建滤波后点云指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // 填入点云数据：设置宽度5，高度1，共5个点
    cloud->width  = 5;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    // 用随机数填充点云坐标，范围[-0.5, 0.5)
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        cloud->points[i].x = rand () / (RAND_MAX + 1.0f) - 0.5;  // 随机x坐标
        cloud->points[i].y = rand () / (RAND_MAX + 1.0f) - 0.5;  // 随机y坐标
        cloud->points[i].z = rand () / (RAND_MAX + 1.0f) - 0.5;  // 随机z坐标
    }

    // 打印滤波前点云
    std::cerr << "滤波前点云: " << std::endl;
    for (size_t i = 0; i < cloud->points.size (); ++i)
        std::cerr << "    " << cloud->points[i].x << " "   // x
                            << cloud->points[i].y << " "   // y
                            << cloud->points[i].z << std::endl;  // z

    // 创建直通滤波器对象
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);             // 设置输入点云
    pass.setFilterFieldName ("z");          // 设置过滤字段为z轴
    pass.setFilterLimits (0.0, 1.0);        // 设置保留范围：0.0 <= z <= 1.0
    // pass.setFilterLimitsNegative (true); // 若取消注释，则保留范围外的点
    pass.filter (*cloud_filtered);          // 执行滤波，结果放入cloud_filtered

    // 打印滤波后点云
    std::cerr << "滤波后点云: " << std::endl;
    for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
        std::cerr << "    " << cloud_filtered->points[i].x << " "   // x
                            << cloud_filtered->points[i].y << " "   // y
                            << cloud_filtered->points[i].z << std::endl;  // z

    return (0);  // 程序正常结束
}