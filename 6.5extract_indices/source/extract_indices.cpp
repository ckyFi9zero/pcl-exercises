#include <iostream>                                    // 引入标准输入输出流
#include <pcl/ModelCoefficients.h>                     // 引入模型系数（平面、球等）
#include <pcl/io/pcd_io.h>                             // 引入PCD文件读写
#include <pcl/point_types.h>                           // 引入PCL点类型
#include <pcl/sample_consensus/method_types.h>         // 引入采样一致性方法类型
#include <pcl/sample_consensus/model_types.h>          // 引入采样一致性模型类型
#include <pcl/segmentation/sac_segmentation.h>         // 引入SAC分割器
#include <pcl/filters/voxel_grid.h>                    // 引入体素网格滤波器
#include <pcl/filters/extract_indices.h>               // 引入根据索引提取/剔除点的滤波器

int main (int argc, char** argv)
{
  // 声明滤波前后的PCL二进制点云指针
  pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2::Ptr cloud_filtered_blob(new pcl::PCLPointCloud2);

  // 声明转换后的XYZ点云指针：滤波后、当前平面、剩余点
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

  // 填入点云数据：读取原始PCD
  pcl::PCDReader reader;
  reader.read("../table_scene_lms400.pcd", *cloud_blob);

  // 打印滤波前点数
  std::cerr << "滤波前的点云: " << cloud_blob->width * cloud_blob->height << " 个数据点。" << std::endl;

  // 创建体素栅格滤波器：下采样，叶大小1 cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;      // 体素栅格对象
  sor.setInputCloud(cloud_blob);                // 设置输入原始点云
  sor.setLeafSize(0.01f, 0.01f, 0.01f);         // 体素尺寸 0.01 m (1 cm)
  sor.filter(*cloud_filtered_blob);             // 完成下采样，结果保存

  // 把二进制格式转换为模板点云 XYZ
  pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

  // 打印滤波后点数
  std::cerr << "滤波后的点云: " << cloud_filtered->width * cloud_filtered->height << " 个数据点。" << std::endl;

  // 保存下采样结果到磁盘
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ>("../table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

  // 平面模型系数与内点索引
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  // 创建分割器对象
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // 可选：优化系数
  seg.setOptimizeCoefficients(true);
  // 必选：平面模型 + RANSAC 方法
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);                   // 最大迭代次数
  seg.setDistanceThreshold(0.01);               // 到平面的最大距离阈值 1 cm

  // 创建索引提取器
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i = 0;                                    // 平面序号
  int nr_points = (int)cloud_filtered->points.size(); // 原始总点数

  // 只要剩余点数 > 30% 原始点数，就继续提取平面
  while (cloud_filtered->points.size() > 0.3 * nr_points)
  {
    // 在当前剩余点云中分割最大平面
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
      // 无法再提取平面
      std::cerr << "无法为给定数据集估计出平面模型。" << std::endl;
      break;
    }

    // 提取内点（属于该平面的点）
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);                 // false 表示保留内点
    extract.filter(*cloud_p);

    // 打印该平面点数（英文→中文）
    std::cerr << "代表平面部分的点云: " << cloud_p->width * cloud_p->height << " 个数据点。" << std::endl;

    // 保存当前平面到文件
    std::stringstream ss;
    ss << "table_scene_lms400_plane_" << i << ".pcd";
    writer.write<pcl::PointXYZ>(ss.str(), *cloud_p, false);

    // 提取剩余点（剔除当前平面）
    extract.setNegative(true);                  // true 表示剔除内点
    extract.filter(*cloud_f);
    cloud_filtered.swap(cloud_f);               // 更新剩余点云
    i++;                                        // 下一个平面序号
  }

  return (0);   // 程序结束
}