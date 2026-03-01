#include <iostream>                                    // 标准输入输出
#include <pcl/console/parse.h>                         // PCL 命令行解析
#include <pcl/filters/extract_indices.h>               // 索引提取滤波器
#include <pcl/io/pcd_io.h>                             // PCD 文件读写
#include <pcl/point_types.h>                           // PCL 点类型定义
#include <pcl/sample_consensus/ransac.h>               // RANSAC 随机采样一致性算法
#include <pcl/sample_consensus/sac_model_plane.h>      // 平面模型
#include <pcl/sample_consensus/sac_model_sphere.h>     // 球体模型
#include <pcl/visualization/pcl_visualizer.h>          // PCL 可视化
#include <boost/thread/thread.hpp>                     // Boost 线程（时间休眠用）

// ------------------------------------------------------------------
// 创建并返回一个极简的 3D 查看器指针
// ------------------------------------------------------------------
boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // 新建查看器，窗口标题为 3D Viewer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (
      new pcl::visualization::PCLVisualizer ("3D viewer"));
  viewer->setBackgroundColor (0, 0, 0);                // 黑色背景
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud"); // 添加点云
  viewer->setPointCloudRenderingProperties (
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud"); // 点大小 3
  viewer->initCameraParameters ();                      // 初始化相机参数
  return viewer;                                        // 返回查看器指针
}

int main(int argc, char** argv)
{
  srand(time(NULL));                                    // 随机种子

  // 创建原始点云和结果点云智能指针
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

  // 生成 5000 个随机点
  cloud->width    = 5000;
  cloud->height   = 1;
  cloud->is_dense = false;                             // 允许 NaN/Inf
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    // 如果命令行带 -s 或 -sf，则生成球面点云并混入 20% 噪声
    if (pcl::console::find_argument (argc, argv, "-s") >= 0 ||
        pcl::console::find_argument (argc, argv, "-sf") >= 0)
    {
      cloud->points[i].x =  rand () / (RAND_MAX + 1.0);  // 0~1 随机
      cloud->points[i].y =  rand () / (RAND_MAX + 1.0);
      if (i % 5 == 0)                                  // 20% 噪声
        cloud->points[i].z =  rand () / (RAND_MAX + 1.0);
      else if(i % 2 == 0)                              // 上半球
        cloud->points[i].z =  sqrt( 1 - cloud->points[i].x*cloud->points[i].x
                                      - cloud->points[i].y*cloud->points[i].y );
      else                                              // 下半球
        cloud->points[i].z = -sqrt( 1 - cloud->points[i].x*cloud->points[i].x
                                        - cloud->points[i].y*cloud->points[i].y );
    }
    // 否则生成含 20% 噪声的平面点云
    else
    {
      cloud->points[i].x =  rand () / (RAND_MAX + 1.0);
      cloud->points[i].y =  rand () / (RAND_MAX + 1.0);
      if( i % 5 == 0)                                  // 20% 噪声
        cloud->points[i].z = rand () / (RAND_MAX + 1.0);
      else                                              // 平面 z = -(x+y)
        cloud->points[i].z = -1 * (cloud->points[i].x + cloud->points[i].y);
    }
  }

  std::vector<int> inliers;                            // 存储 RANSAC 内点索引

  // 创建两种模型：球体和平面
  pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
      model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
      model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));

  // 根据命令行参数选择模型并运行 RANSAC
  if(pcl::console::find_argument (argc, argv, "-f") >= 0)      // -f 用平面
  {
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (0.01);              // 距离阈值 1 cm
    ransac.computeModel();                           // 拟合
    ransac.getInliers(inliers);                      // 获取内点索引
  }
  else if (pcl::console::find_argument (argc, argv, "-sf") >= 0 ) // -sf 用球体
  {
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
    ransac.setDistanceThreshold (0.01);
    ransac.computeModel();
    ransac.getInliers(inliers);
  }

  // 根据内点索引复制出最终点云
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

  // 根据是否使用 RANSAC 决定显示全部点云还是仅显示内点
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  if (pcl::console::find_argument (argc, argv, "-f") >= 0 ||
      pcl::console::find_argument (argc, argv, "-sf") >= 0)
    viewer = simpleVis(final);        // 仅显示内点
  else
    viewer = simpleVis(cloud);        // 显示原始点云

  // 主循环：持续更新视图直到用户关闭窗口
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);                               // 处理事件
    boost::this_thread::sleep (boost::posix_time::microseconds (100000)); // 休眠 100 ms
  }
  return 0;
}