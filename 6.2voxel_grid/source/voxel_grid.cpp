#include <iostream>                            // 引入标准输入输出流
#include <pcl/io/pcd_io.h>                     // 引入PCD文件读写功能
#include <pcl/point_types.h>                   // 引入PCL点类型定义
#include <pcl/filters/voxel_grid.h>            // 引入体素网格滤波器头文件

int main (int argc, char** argv)
{
  // 创建原始点云（二进制格式 PCLPointCloud2）
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
  // 创建滤波后点云（二进制格式）
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2());

  // 填入点云数据：使用PCDReader读取文件
  pcl::PCDReader reader;
  // 读取 PCD 文件到 cloud；需提前下载 table_scene_lms400.pcd 并放到上级目录
  reader.read ("../table_scene_lms400.pcd", *cloud);

  // 打印滤波前点云总点数（宽×高）及字段列表
  std::cerr << "滤波前的点云: " << cloud->width * cloud->height 
            << " 数据点 (" << pcl::getFieldsList (*cloud) << ").";

  // 创建体素网格滤波器对象
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);                  // 设置输入点云
  sor.setLeafSize (0.01f, 0.01f, 0.01f);    // 设置体素大小为 1 cm × 1 cm × 1 cm
  sor.filter (*cloud_filtered);               // 执行滤波，结果放入 cloud_filtered

  // 打印滤波后点云总点数及字段列表
  std::cerr << "滤波后的点云: " << cloud_filtered->width * cloud_filtered->height 
            << " 数据点 (" << pcl::getFieldsList (*cloud_filtered) << ").";

  // 创建PCDWriter写入滤波后点云
  pcl::PCDWriter writer;
  // 将滤波后的点云保存为 2f.pcd；保留原坐标系和旋转（单位四元数）
  writer.write ("../2f.pcd", *cloud_filtered, 
                Eigen::Vector4f::Zero (),       // 平移向量 (0,0,0)
                Eigen::Quaternionf::Identity (), // 无旋转
                false);                          // 不保存二进制压缩

  return 0;  // 程序正常结束
}