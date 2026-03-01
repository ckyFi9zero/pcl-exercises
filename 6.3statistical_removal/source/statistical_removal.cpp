#include <iostream>                                     // 引入标准输入输出流
#include <pcl/io/pcd_io.h>                              // 引入PCD文件读写功能
#include <pcl/point_types.h>                            // 引入PCL点类型定义
#include <pcl/filters/statistical_outlier_removal.h>    // 引入统计离群点移除滤波器头文件

int main (int argc, char** argv)
{
  // 创建原始点云指针
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  // 创建滤波后点云指针
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // 填入点云数据：使用PCDReader读取文件
  pcl::PCDReader reader;
  // 读取 PCD 文件到 cloud；需提前下载 table_scene_lms400.pcd 并放到上级目录
  reader.read<pcl::PointXYZ> ("../table_scene_lms400.pcd", *cloud);

  // 打印滤波前点云信息
  std::cerr << "滤波前的点云: " << std::endl;
  std::cerr << *cloud << std::endl;

  // 创建统计离群点移除滤波器对象
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);              // 设置输入点云
  sor.setMeanK (50);                      // 设置用于计算平均距离的邻域点数（K=50）
  sor.setStddevMulThresh (1.0);           // 设置标准差倍数阈值（1.0倍标准差以外视为离群点）
  sor.filter (*cloud_filtered);           // 执行滤波，保留内点，结果放入 cloud_filtered

  // 打印滤波后内点点云信息
  std::cerr << "滤波后的点云: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  // 创建PCDWriter写入滤波后内点点云
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("../table_scene_lms400_inliers.pcd", *cloud_filtered, false);

  // 设置滤波器为反向选择，提取离群点（外点）
  sor.setNegative (true);
  sor.filter (*cloud_filtered);           // 再次滤波，结果放入 cloud_filtered（此时为外点）

  // 保存离群点（外点）到文件
  writer.write<pcl::PointXYZ> ("../table_scene_lms400_outliers.pcd", *cloud_filtered, false);

  return (0);  // 程序正常结束
}