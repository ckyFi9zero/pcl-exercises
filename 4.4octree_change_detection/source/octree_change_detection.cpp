#include <pcl/point_cloud.h>                      // 引入PCL点云数据结构
#include <pcl/octree/octree.h>                    // 引入PCL八叉树相关功能
#include <iostream>                               // 引入标准输入输出流
#include <vector>                                 // 引入STL向量容器
#include <ctime>                                  // 引入时间函数，用于随机种子

int
main (int argc, char**argv)
{
    srand ((unsigned int) time (NULL));           // 用当前时间初始化随机数种子

    // 八叉树分辨率（体素大小）设为32
    float resolution = 32.0f;

    // 创建八叉树“变化检测”对象，用于比较两个点云差异
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution);

    // 创建第一个点云指针 cloudA
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);

    // 设置cloudA：共128个点，宽度128，高度1（无序点云）
    cloudA->width = 128;
    cloudA->height = 1;
    cloudA->points.resize(cloudA->width * cloudA->height);

    // 用随机坐标填充cloudA，范围[0,64)
    for (size_t i = 0; i < cloudA->points.size(); ++i)
    {
        cloudA->points[i].x = 64.0f * rand() / (RAND_MAX + 1.0f);
        cloudA->points[i].y = 64.0f * rand() / (RAND_MAX + 1.0f);
        cloudA->points[i].z = 64.0f * rand() / (RAND_MAX + 1.0f);
    }

    // 把cloudA设为八叉树输入，并添加所有点建立八叉树
    octree.setInputCloud(cloudA);
    octree.addPointsFromInputCloud();

    // 点云A是我们的参考点云，用八叉树结构描述它的空间分布。
    // OctreePointCloudChangeDetector类继承自Octree2BufBase类，
    // 后者允许同时在内存中保存和管理两棵八叉树。
    // 此外，它实现了一个内存池，可以重用已经分配的节点对象，因此在生成多个点云的八叉树时减少了昂贵的内存分配和回收操作。
    // 通过调用octree. switchbuffers()，我们重置了八叉树类，同时在内存中保留了之前的八叉树结构。

    // 交换八叉树缓存：把cloudA对应的八叉树存入“前缓存”，准备接收新点云
    octree.switchBuffers();

    // 创建第二个点云指针 cloudB
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);

    // 设置cloudB：同样128个随机点
    cloudB->width = 128;
    cloudB->height = 1;
    cloudB->points.resize(cloudB->width * cloudB->height);

    // 用随机坐标填充cloudB，范围[0,64)
    for (size_t i = 0; i < cloudB->points.size(); ++i)
    {
        cloudB->points[i].x = 64.0f * rand() / (RAND_MAX + 1.0f);
        cloudB->points[i].y = 64.0f * rand() / (RAND_MAX + 1.0f);
        cloudB->points[i].z = 64.0f * rand() / (RAND_MAX + 1.0f);
    }

    // 把cloudB设为八叉树输入，并添加所有点（存到“当前缓存”）
    octree.setInputCloud(cloudB);
    octree.addPointsFromInputCloud();

    // 存放“新增点”索引的向量
    std::vector<int> newPointIdxVector;
    // 为了检索存储在当前八叉树结构体素(基于cloudB)中的点，这些点在之前的八叉树结构(基于cloudA)中不存在，我们可以调用getPointIndicesFromNewVoxels方法，它返回结果点索引的向量。
    // 调用变化检测：获取cloudB中位于新体素（cloudA没有）的点的索引
    octree.getPointIndicesFromNewVoxels(newPointIdxVector);

    // 打印检测结果
    std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
    for (size_t i = 0; i < newPointIdxVector.size(); ++i)
        std::cout << i << "# Index:" << newPointIdxVector[i]
                  << "  Point:" << cloudB->points[newPointIdxVector[i]].x << " "
                  << cloudB->points[newPointIdxVector[i]].y << " "
                  << cloudB->points[newPointIdxVector[i]].z << std::endl;

    return 0;  // 程序正常结束
}