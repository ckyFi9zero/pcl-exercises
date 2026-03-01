#include <pcl/range_image/range_image.h>                // PCL 距离图像头文件
#include <pcl/range_image/range_image_planar.h>         // PCL 平面距离图像头文件
#include <pcl/io/io.h>                                  // PCL IO 通用接口
#include <pcl/io/pcd_io.h>                              // PCL PCD 文件读写
#include <pcl/features/integral_image_normal.h>         // 积分图法线估计
#include <pcl/visualization/cloud_viewer.h>             // 点云可视化
#include <pcl/point_types.h>                            // PCL 点类型定义
#include <pcl/features/normal_3d.h>                     // 3D 法线估计
#include <pcl/console/print.h>                          // 控制台彩色打印
#include <pcl/surface/organized_fast_mesh.h>            // 有序点云快速网格化
#include <pcl/console/time.h>                           // 控制台计时器
#include <Eigen/StdVector>                              // Eigen 标准容器
#include <Eigen/Geometry>                               // Eigen 几何模块
#include <iostream>                                     // 标准输入输出
#include <pcl/surface/impl/organized_fast_mesh.hpp>     // 有序网格实现
#include <boost/thread/thread.hpp>                      // boost 线程

#include <pcl/common/common_headers.h>                  // PCL 公共头文件

#include <pcl/visualization/range_image_visualizer.h>   // 距离图像可视化
#include <pcl/visualization/pcl_visualizer.h>           // PCL 可视化器
#include <pcl/console/parse.h>                          // 命令行解析
using namespace pcl::console;                           // 使用 pcl::console 命名空间

int main (int argc, char** argv) {

    // 检查参数
    if (argc < 2)
    {
        // 打印错误信息（中文翻译）
        print_error ("语法错误: %s input.pcd -w 640 -h 480 -cx 320 -cy 240 -fx 525 -fy 525 -type 0 -size 2\n", argv[0]);
        print_info ("  可用选项:\n");
        print_info ("                     -w X = 深度图像宽度 ");
        return -1;
    }

    // 读取输入文件名
    std::string filename = argv[1];

    // 默认参数
    int width = 640, height = 480, size = 2, type = 0;
    float fx = 525, fy = 525, cx = 320, cy = 240;

    // 解析命令行参数
    parse_argument (argc, argv, "-w", width);      // 图像宽度
    parse_argument (argc, argv, "-h", height);     // 图像高度
    parse_argument (argc, argv, "-cx", cx);        // 主点 x
    parse_argument (argc, argv, "-cy", cy);        // 主点 y
    parse_argument (argc, argv, "-fx", fx);        // 焦距 fx
    parse_argument (argc, argv, "-fy", fy);        // 焦距 fy
    parse_argument (argc, argv, "-type", type);    // 三角化类型
    parse_argument (argc, argv, "-size", size);    // 三角像素尺寸

    // 读取 PCD 文件
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile (filename, *cloud);

    // 输出提示（中文翻译）
    print_info ("成功读取 PCD 文件\n");

    // 设置传感器位姿为单位矩阵
    Eigen::Affine3f sensorPose;
    sensorPose.setIdentity(); 

    // 坐标系为相机坐标系
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel = 0.00;
    float minRange = 0.0f;

    // 创建平面距离图像
    pcl::RangeImagePlanar::Ptr rangeImage(new pcl::RangeImagePlanar);
    rangeImage->createFromPointCloudWithFixedSize(
        *cloud, width, height, cx, cy, fx, fy, sensorPose, coordinate_frame);

    // 打印距离图像指针地址
    std::cout << rangeImage << "\n";

    // 可视化距离图像
    pcl::visualization::RangeImageVisualizer range_image_widget ("点云库PCL从入门到精通");
    range_image_widget.showRangeImage (*rangeImage);
    range_image_widget.setWindowTitle("点云库PCL从入门到精通");

    // 基于距离图像进行三角化
    pcl::OrganizedFastMesh<pcl::PointWithRange>::Ptr tri(new pcl::OrganizedFastMesh<pcl::PointWithRange>);
    pcl::search::KdTree<pcl::PointWithRange>::Ptr tree (new pcl::search::KdTree<pcl::PointWithRange>);
    tree->setInputCloud(rangeImage);  // 设置搜索点云
    pcl::PolygonMesh triangles;       // 存储结果网格

    // 设置网格化参数
    tri->setTrianglePixelSize(size);  // 设置三角像素尺寸
    tri->setInputCloud(rangeImage);   // 输入距离图像
    tri->setSearchMethod(tree);       // 设置搜索方法
    tri->setTriangulationType((pcl::OrganizedFastMesh<pcl::PointWithRange>::TriangulationType)type);
    tri->reconstruct(triangles);      // 执行重建

    // 创建 3D 可视化窗口
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (
        new pcl::visualization::PCLVisualizer ("3D 查看器"));
    viewer->setBackgroundColor(0.5, 0.5, 0.5);  // 灰色背景
    viewer->addPolygonMesh(triangles, "tin");     // 添加网格
    viewer->addCoordinateSystem();                // 添加坐标系

    // 主循环，保持窗口打开
    while (!range_image_widget.wasStopped() && !viewer->wasStopped())
    {
        range_image_widget.spinOnce();  // 刷新距离图像窗口
        pcl_sleep(0.01);                // 短暂休眠
        viewer->spinOnce();             // 刷新 3D 窗口
    }
}