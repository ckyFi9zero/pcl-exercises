#include <pcl/point_cloud.h>          // 引入PCL点云数据结构
#include <pcl/octree/octree.h>        // 引入PCL八叉树（octree）相关功能
#include <iostream>                   // 引入标准输入输出流
#include <vector>                     // 引入STL向量容器
#include <ctime>                      // 引入时间函数，用于随机种子

int
main (int argc, char**argv)
{
    srand ((unsigned int) time (NULL));               // 用当前时间初始化随机数种子
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 创建PointXYZ类型的点云指针

    // 设置点云大小：共1000个点，宽度1000，高度1（无序点云）
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    // 用随机数填充点云坐标，范围[0,1024)
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f); // 随机x坐标
        cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f); // 随机y坐标
        cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f); // 随机z坐标
    }

    float resolution = 128.0f;  // 设置八叉树体素（voxel）分辨率为128
    // 创建八叉树搜索对象，指定分辨率
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);

    octree.setInputCloud(cloud);         // 把点云设置为八叉树输入
    octree.addPointsFromInputCloud();    // 把点云中所有点添加进八叉树结构

    // 随机生成一个待查询点
    pcl::PointXYZ searchPoint;
    searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

    // 体素内近邻搜索：查找与searchPoint位于同一voxel的所有点
    std::vector<int> pointIdxVec;   // 存放体素内点的索引
    if (octree.voxelSearch(searchPoint, pointIdxVec))  // 如果找到
    {
        std::cout << "Neighbors within voxel search at (" << searchPoint.x
                  << " " << searchPoint.y
                  << " " << searchPoint.z << ")" << std::endl;
        // 逐个输出体素内点的坐标
        for (size_t i = 0; i < pointIdxVec.size(); ++i)
            std::cout << "    " << cloud->points[pointIdxVec[i]].x << " "
                      << cloud->points[pointIdxVec[i]].y << " "
                      << cloud->points[pointIdxVec[i]].z << std::endl;
    }

    // K近邻搜索：查找离searchPoint最近的K个点
    int K = 10;                                   // 设置K=10
    std::vector<int>   pointIdxNKNSearch;         // 存放K个近邻的索引
    std::vector<float> pointNKNSquaredDistance;   // 存放对应的平方距离

    std::cout << "K nearest neighbor search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with K=" << K << std::endl;
    // 执行K近邻搜索，返回找到的点数
    if (octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        // 输出每个近邻点的坐标和平方距离
        for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
            std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x << " "
                      << cloud->points[pointIdxNKNSearch[i]].y << " "
                      << cloud->points[pointIdxNKNSearch[i]].z
                      << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
    }

    // 半径内近邻搜索：查找以searchPoint为中心、给定半径内的所有点
    std::vector<int>   pointIdxRadiusSearch;        // 存放半径内点的索引
    std::vector<float> pointRadiusSquaredDistance;  // 存放对应平方距离
    float radius = 256.0f * rand() / (RAND_MAX + 1.0f); // 随机半径 [0,256)

    std::cout << "Neighbors within radius search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with radius=" << radius << std::endl;
    // 执行半径搜索，返回找到的点数
    if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        // 输出半径内每个点的坐标和平方距离
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
            std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x << " "
                      << cloud->points[pointIdxRadiusSearch[i]].y << " "
                      << cloud->points[pointIdxRadiusSearch[i]].z
                      << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    }

    return 0;  // 程序正常结束
}