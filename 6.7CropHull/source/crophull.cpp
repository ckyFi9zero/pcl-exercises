#include <pcl/visualization/cloud_viewer.h>    // 引入可视化 CloudViewer
#include <iostream>                              // 标准输入输出
#include <pcl/io/io.h>                           // PCL 通用 IO
#include <pcl/io/pcd_io.h>                       // PCD 读写
#include <vector>                                // STL 向量
#include <pcl/point_cloud.h>                     // 点云模板
#include <pcl/point_types.h>                     // 点类型
#include <pcl/io/pcd_io.h>                       // 重复包含，可忽略
#include <pcl/filters/crop_hull.h>               // 基于凸包裁剪滤波
#include <pcl/surface/concave_hull.h>            // 凸包/凹包生成

int main(int argc, char** argv)
{
    // 创建输入点云指针并读取文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read(argv[1], *cloud);               // 从命令行参数读取 PCD

    // 构建二维凸包边界点（共 5 个）
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundingbox_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    boundingbox_ptr->push_back(pcl::PointXYZ(0.1, 0.1, 0));    // 点1
    boundingbox_ptr->push_back(pcl::PointXYZ(0.1, -0.1, 0));   // 点2
    boundingbox_ptr->push_back(pcl::PointXYZ(-0.1, 0.1, 0));   // 点3
    boundingbox_ptr->push_back(pcl::PointXYZ(-0.1, -0.1, 0));  // 点4
    boundingbox_ptr->push_back(pcl::PointXYZ(0.15, 0.1, 0));   // 点5

    // 利用 ConvexHull 计算二维凸包
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(boundingbox_ptr);        // 输入边界点
    hull.setDimension(2);                       // 2D 凸包
    std::vector<pcl::Vertices> polygons;        // 存储多边形顶点索引
    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
    hull.reconstruct(*surface_hull, polygons);  // 生成凸包顶点与面

    // 使用 CropHull 对原始点云进行裁剪
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropHull<pcl::PointXYZ> bb_filter;
    bb_filter.setDim(2);                        // 2D 裁剪
    bb_filter.setInputCloud(cloud);             // 待裁剪点云
    bb_filter.setHullIndices(polygons);         // 凸包面索引
    bb_filter.setHullCloud(surface_hull);       // 凸包顶点
    bb_filter.filter(*objects);                 // 执行裁剪，结果存入 objects

    // 终端输出：裁剪后剩余点数（英文→中文）
    std::cout << "裁剪后剩余点数: " << objects->size() << std::endl;

    // 创建三视窗可视化器
    boost::shared_ptr<pcl::visualization::PCLVisualizer> for_visualizer_v(
        new pcl::visualization::PCLVisualizer("crophull display"));
    for_visualizer_v->setBackgroundColor(255, 255, 255);  // 全局白色背景

    // 视口1：原始点云 + 凸包多边形
    int v1(0);
    for_visualizer_v->createViewPort(0.0, 0.0, 0.33, 1, v1);
    for_visualizer_v->setBackgroundColor(255, 255, 255, v1);
    for_visualizer_v->addPointCloud(cloud, "cloud", v1);
    for_visualizer_v->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "cloud");   // 红色
    for_visualizer_v->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    for_visualizer_v->addPolygon<pcl::PointXYZ>(
        surface_hull, 0, 0.069*255, 0.2*255, "backview_hull_polyline1", v1);

    // 视口2：凸包顶点 + 多边形
    int v2(0);
    for_visualizer_v->createViewPort(0.33, 0.0, 0.66, 1, v2);
    for_visualizer_v->setBackgroundColor(255, 255, 255, v2);
    for_visualizer_v->addPointCloud(surface_hull, "surface_hull", v2);
    for_visualizer_v->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "surface_hull");
    for_visualizer_v->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "surface_hull");
    for_visualizer_v->addPolygon<pcl::PointXYZ>(
        surface_hull, 0, 0.069*255, 0.2*255, "backview_hull_polyline", v2);

    // 视口3：裁剪结果（保留在凸包内的点）
    int v3(0);
    for_visualizer_v->createViewPort(0.66, 0.0, 1.0, 1, v3);
    for_visualizer_v->setBackgroundColor(255, 255, 255, v3);
    for_visualizer_v->addPointCloud(objects, "objects", v3);
    for_visualizer_v->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "objects");
    for_visualizer_v->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "objects");

    // 主循环：保持窗口刷新
    while (!for_visualizer_v->wasStopped())
    {
        for_visualizer_v->spinOnce(1000);   // 每 1000 ms 刷新一次
    }
    system("pause");    // Windows 下暂停，方便查看结果
}