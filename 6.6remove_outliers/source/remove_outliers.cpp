#include <iostream>                                    // 引入标准输入输出
#include <pcl/point_types.h>                           // 引入点类型定义
#include <pcl/filters/radius_outlier_removal.h>        // 引入半径离群点移除滤波器
#include <pcl/filters/conditional_removal.h>           // 引入条件滤波器

int main (int argc, char** argv)
{
  // 检查命令行参数：必须输入 -r 或 -c
  if (argc != 2)
  {
    std::cerr << "请在命令行参数中指定 '-r' 或 '-c'" << std::endl;
    exit(0);
  }

  // 创建原始点云指针
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // 创建滤波后点云指针
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

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

  // 根据命令行参数选择滤波方式
  if (strcmp(argv[1], "-r") == 0) {
    // 半径离群点移除滤波器
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud);              // 输入点云
    outrem.setRadiusSearch(0.8);              // 搜索半径 0.8
    outrem.setMinNeighborsInRadius(2);        // 半径内最少邻居数 2
    outrem.filter(*cloud_filtered);           // 执行滤波
  }
  else if (strcmp(argv[1], "-c") == 0) {
    // 条件滤波：只保留 z 在 (0, 0.8) 之间的点
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(
        new pcl::ConditionAnd<pcl::PointXYZ>());
    // 添加条件：z > 0
    range_cond->addComparison(
        pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));
    // 添加条件：z < 0.8
    range_cond->addComparison(
        pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.8)));

    // 创建条件滤波器
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond);         // 设置条件
    condrem.setInputCloud(cloud);             // 输入点云
    condrem.setKeepOrganized(true);           // 保持点云有序（保留 NaN）
    condrem.filter(*cloud_filtered);          // 执行滤波
  }
  else {
    std::cerr << "请在命令行参数中指定 '-r' 或 '-c'" << std::endl;
    exit(0);
  }

  // 打印滤波前点云（英文→中文）
  std::cerr << "滤波前的点云：" << std::endl;
  for (size_t i = 0; i < cloud->points.size(); ++i)
    std::cerr << "    " << cloud->points[i].x << " "
                        << cloud->points[i].y << " "
                        << cloud->points[i].z << std::endl;

  // 打印滤波后点云（英文→中文）
  std::cerr << "滤波后的点云：" << std::endl;
  for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " "
                        << cloud_filtered->points[i].y << " "
                        << cloud_filtered->points[i].z << std::endl;

  return (0);  // 程序结束
}