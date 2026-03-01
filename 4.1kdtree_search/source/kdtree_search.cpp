#include <pcl/point_cloud.h>          // 引入PCL点云头文件
#include <pcl/kdtree/kdtree_flann.h>  // 引入PCL基于FLANN的KD-Tree头文件
#include <iostream>                   // 引入标准输入输出流
#include <vector>                     // 引入STL向量容器
#include <ctime>                      // 引入时间相关函数，用于随机数种子

int main (int argc, char**argv)
{
    srand (time (NULL));              // 用当前时间初始化随机数种子

    // 创建PointXYZ类型的点云指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // 设置点云基本属性：宽度1000，高度1，即1000个无序点
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    // 用随机数填充点云
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);  // 随机x坐标 [0,1024)
        cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);  // 随机y坐标 [0,1024)
        cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);  // 随机z坐标 [0,1024)
    }

    // 创建KD-Tree对象，并把点云设置为输入
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // 随机生成一个待查询点
    pcl::PointXYZ searchPoint;
    searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

    // K近邻搜索：寻找离searchPoint最近的K个点
    int K = 10;                                            // 设定K=10
    std::vector<int>   pointIdxNKNSearch(K);               // 存放近邻点索引
    std::vector<float> pointNKNSquaredDistance(K);         // 存放对应近邻点的平方距离

    std::cout << "K近邻搜索 (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") K为" << K << std::endl;

    // 执行K近邻搜索，返回找到的近邻点数目
    if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        // 打印每个近邻点的坐标及平方距离
        for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
            std::cout << "    "
                      << cloud->points[pointIdxNKNSearch[i]].x << " "
                      << cloud->points[pointIdxNKNSearch[i]].y << " "
                      << cloud->points[pointIdxNKNSearch[i]].z
                      << " (平方距离: " << pointNKNSquaredDistance[i] << ")" << std::endl;
    }

    // 半径搜索：寻找以searchPoint为中心、半径r内的所有点
    std::vector<int>   pointIdxRadiusSearch;           // 存放半径内点索引
    std::vector<float> pointRadiusSquaredDistance;     // 存放对应点的平方距离
    float radius = 256.0f * rand() / (RAND_MAX + 1.0f); // 随机半径 [0,256)

    std::cout << "半径搜索 (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") 半径为" << radius << std::endl;

    // 执行半径搜索，返回找到的点的数目
    if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        // 打印半径内每个点的坐标及平方距离
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
            std::cout << "    "
                      << cloud->points[pointIdxRadiusSearch[i]].x << " "
                      << cloud->points[pointIdxRadiusSearch[i]].y << " "
                      << cloud->points[pointIdxRadiusSearch[i]].z
                      << " (半径为: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    }

    return 0;  // 程序正常结束
}