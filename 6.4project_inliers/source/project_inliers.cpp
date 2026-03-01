#include <iostream>                                    // 引入标准输入输出流
#include <pcl/io/pcd_io.h>                             // 引入 PCD 读写
#include <pcl/point_types.h>                           // 引入 PCL 点类型
#include <pcl/ModelCoefficients.h>                     // 引入模型系数
#include <pcl/filters/project_inliers.h>               // 引入投影滤波器

int main (int argc, char** argv)
{
  // 创建原始点云指针
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // 创建投影后点云指针
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

  // 生成 5 个随机点
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);  // 随机 x
    cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);  // 随机 y
    cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);  // 随机 z
  }

  // 输出原始点云
  std::cerr << "投影前的点云：" << std::endl;
  for (size_t i = 0; i < cloud->points.size(); ++i)
    std::cerr << "    " << cloud->points[i].x << " "
                        << cloud->points[i].y << " "
                        << cloud->points[i].z << std::endl;

  // 定义平面模型系数：X=Y=0, Z=1, d=0 → 即平面 z = 0
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  coefficients->values.resize(4);
  coefficients->values[0] = coefficients->values[1] = 0;  // X 系数
  coefficients->values[2] = 1.0;                         // Z 系数
  coefficients->values[3] = 0;                           // 常数 d

  // 创建投影滤波器
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);     // 模型类型：平面
  proj.setInputCloud(cloud);                  // 输入点云
  proj.setModelCoefficients(coefficients);    // 平面参数
  proj.filter(*cloud_projected);              // 执行投影

  // 输出投影后点云
  std::cerr << "投影后的点云：" << std::endl;
  for (size_t i = 0; i < cloud_projected->points.size(); ++i)
    std::cerr << "    " << cloud_projected->points[i].x << " "
                        << cloud_projected->points[i].y << " "
                        << cloud_projected->points[i].z << std::endl;

  return (0);  // 程序结束
}