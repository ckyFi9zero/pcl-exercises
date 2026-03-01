// 标准输入输出
#include <iostream>
// PCL 命令行解析工具（未使用）
#include <pcl/console/parse.h>
// 点云索引提取器
#include <pcl/filters/extract_indices.h>
// PCD 文件读写
#include <pcl/io/pcd_io.h>
// 点类型定义
#include <pcl/point_types.h>

// PCL 可视化工具
#include <pcl/visualization/pcl_visualizer.h>
// Boost 多线程支持（用于可视化）
#include <boost/thread/thread.hpp>
// 边界检测头文件
#include <pcl/features/boundary.h>
// 数学常量（如 M_PI）
#include <math.h>
// Boost 智能指针辅助
#include <boost/make_shared.hpp>
// 重复包含（冗余）
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

// 范围图像可视化（未使用）
#include <pcl/visualization/range_image_visualizer.h>
// 法向量估计
#include <pcl/features/normal_3d.h>

// 重复包含（冗余）
#include <pcl/visualization/pcl_visualizer.h>
// 协方差采样（未使用）
#include <pcl/filters/covariance_sampling.h>
// 法线空间滤波（未使用）
#include <pcl/filters/normal_space.h>
// KD-Tree 搜索
#include <pcl/kdtree/kdtree_flann.h>
// 再次包含边界检测（冗余）
#include <pcl/features/boundary.h>
// PLY 文件读取（未使用）
#include <pcl/io/ply_io.h>


// 函数：估计点云边界
int estimateBorders(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float re, float reforn) 
{ 
  // 存储每个点的边界标志（0:非边界, 1:边界）
  pcl::PointCloud<pcl::Boundary> boundaries; 
  
  // 创建边界估计器
  pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst; 
  
  // 创建法向量估计器
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst; 
  
  // 存储法向量的指针
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); 
  
  // 存储检测到的边界点
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>()); 

  // 设置法向量估计的输入点云
  normEst.setInputCloud(cloud);  // 注意：这里可以简化，不需要再构造 Ptr

  // 设置法向量估计的搜索半径
  normEst.setRadiusSearch(reforn); 

  // 计算法向量
  normEst.compute(*normals); 

  // 设置边界估计器的输入点云
  boundEst.setInputCloud(cloud); 

  // 设置边界估计器的输入法向量
  boundEst.setInputNormals(normals); 

  // 设置边界检测的搜索半径
  boundEst.setRadiusSearch(re); 

  // 设置角度阈值：大于 π/4 (45°) 视为边界
  boundEst.setAngleThreshold(M_PI / 4); 

  // 设置搜索方法：使用 KD-Tree
  boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>())); 

  // 执行边界检测
  boundEst.compute(boundaries); 

  // 遍历所有点，提取边界点
  for (int i = 0; i < cloud->points.size(); i++) 
  { 
    // 如果该点是边界点
    if (boundaries[i].boundary_point > 0) 
    { 
      // 添加到边界点云
      cloud_boundary->push_back(cloud->points[i]); 
    } 
  } 

  // 创建可视化窗口
  boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("boundary"));

  int v1(0); 
  // 创建左视口（0~50% 宽度）
  MView->createViewPort(0.0, 0.0, 0.5, 1.0, v1); 
  MView->setBackgroundColor(0.3, 0.3, 0.3, v1); 
  MView->addText("Raw point clouds", 10, 10, "v1_text", v1); 

  int v2(0); 
  // 创建右视口（50%~100% 宽度）
  MView->createViewPort(0.5, 0.0, 1.0, 1.0, v2); 
  MView->setBackgroundColor(0.5, 0.5, 0.5, v2); 
  MView->addText("Boundary point clouds", 10, 10, "v2_text", v2); 

  // 左视口显示原始点云
  MView->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud", v1);
  // 右视口显示边界点云
  MView->addPointCloud<pcl::PointXYZ>(cloud_boundary, "cloud_boundary", v2);

  // 设置原始点云为红色
  MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
  // 设置边界点云为绿色
  MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_boundary", v2);

  // 添加坐标系
  MView->addCoordinateSystem(1.0);
  // 初始化相机
  MView->initCameraParameters();

  // 启动可视化（阻塞）
  MView->spin();

  return 0; 
} 


int main(int argc, char** argv)
{
  // 初始化随机种子（未使用）
  srand(time(NULL));

  float re, reforn;           // 搜索半径
  re = std::atof(argv[2]);    // 边界检测半径
  reforn = std::atof(argv[3]); // 法向量估计半径

  // 创建源点云指针
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>); 

  // 从 PCD 文件加载点云
  pcl::io::loadPCDFile(argv[1], *cloud_src);	

  // 调用边界检测函数
  estimateBorders(cloud_src, re, reforn);

  return 0;
}