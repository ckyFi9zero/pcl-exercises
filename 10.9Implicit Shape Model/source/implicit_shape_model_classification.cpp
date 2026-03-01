// 包含头文件（同上）
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/feature.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/impl/fpfh.hpp>
#include <pcl/recognition/implicit_shape_model.h>
#include <pcl/recognition/impl/implicit_shape_model.hpp> // 包含 ISM 实现

int main (int argc, char** argv)
{
  // 检查命令行参数
  if (argc == 0) // ? Bug: 应为 argc < 3
  { 
    std::cout << std::endl;
    std::cout << "Usage: " << argv[0] << " test_scene.pcd class1_label(int)" << std::endl << std::endl;
    std::cout << "Where the parameter class1_label is the object you want to be segmented and recognized" << std::endl << std::endl;
    return (-1);
  }

  // 创建法向量估计器
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setRadiusSearch (25.0);
  // 创建 FPFH 特征估计器
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> >::Ptr fpfh
    (new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> >);
  fpfh->setRadiusSearch (30.0);
  // 创建特征估计器基类指针
  pcl::Feature< pcl::PointXYZ, pcl::Histogram<153> >::Ptr feature_estimator(fpfh);

  // 创建 ISM 估计器
  pcl::ism::ImplicitShapeModelEstimation<153, pcl::PointXYZ, pcl::Normal> ism;
  ism.setFeatureEstimator(feature_estimator); // 设置特征估计器
  ism.setSamplingSize (2.0f);                 // 设置采样大小

  // 创建 ISM 模型指针
  pcl::ism::ImplicitShapeModelEstimation<153, pcl::PointXYZ, pcl::Normal>::ISMModelPtr model = 
    boost::shared_ptr<pcl::features::ISMModel> (new pcl::features::ISMModel);
  // 从文件加载训练好的模型
  std::string file ("trained_ism_model.txt");
  model->loadModelFromfile (file);

  // 获取要识别的类别标签
  unsigned int testing_class = static_cast<unsigned int> (strtol (argv[2], 0, 10));
  // 创建测试点云指针
  pcl::PointCloud<pcl::PointXYZ>::Ptr testing_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // 从文件加载测试点云
  if ( pcl::io::loadPCDFile <pcl::PointXYZ> (argv[1], *testing_cloud) == -1 )
    return (-1);

  // 为测试点云计算法向量
  pcl::PointCloud<pcl::Normal>::Ptr testing_normals = (new pcl::PointCloud<pcl::Normal>)->makeShared ();
  normal_estimator.setInputCloud (testing_cloud);
  normal_estimator.compute (*testing_normals);

  // 使用 ISM 模型在测试场景中寻找指定类别的物体
  boost::shared_ptr<pcl::features::ISMVoteList<pcl::PointXYZ> > vote_list = ism.findObjects (
    model,               // 训练好的模型
    testing_cloud,       // 测试点云
    testing_normals,     // 测试法向量
    testing_class);      // 要识别的类别

  // 从模型中获取该类别的特征尺度（sigma）
  double radius = model->sigmas_[testing_class] * 10.0; // 搜索半径
  double sigma = model->sigmas_[testing_class];         // 高斯核的标准差
  // 存储找到的最强峰值（物体中心）
  std::vector<pcl::ISMPeak, Eigen::aligned_allocator<pcl::ISMPeak> > strongest_peaks;
  // 在投票列表中查找最强的峰值
  vote_list->findStrongestPeaks (strongest_peaks, testing_class, radius, sigma);

  // 创建一个用于可视化的彩色点云，用于显示检测到的物体中心
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();
  colored_cloud->height = 0;
  colored_cloud->width = 1;

  pcl::PointXYZRGB point;
  point.r = 255;
  point.g = 255;
  point.b = 255;

  // （被注释）将测试点云也加入彩色点云（可选）
  /*
  for (size_t i_point = 0; i_point < testing_cloud->points.size (); i_point++)
  {
    point.x = testing_cloud->points[i_point].x;
    point.y = testing_cloud->points[i_point].y;
    point.z = testing_cloud->points[i_point].z;
    colored_cloud->points.push_back (point);
  }
  colored_cloud->height += testing_cloud->points.size ();
  */

  // 将检测到的物体中心（峰值）加入彩色点云
  point.r = 255;
  point.g = 0;
  point.b = 0;
  for (size_t i_vote = 0; i_vote < strongest_peaks.size (); i_vote++)
  {
    point.x = strongest_peaks[i_vote].x;
    point.y = strongest_peaks[i_vote].y;
    point.z = strongest_peaks[i_vote].z;
    colored_cloud->points.push_back (point);
  }
  colored_cloud->height += strongest_peaks.size (); // 更新高度

  // 创建可视化窗口
  pcl::visualization::PCLVisualizer viewer ("implicit shape model");
  viewer.setBackgroundColor(1,1,1); // 白色背景
  // 为测试点云设置绿色
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorh(testing_cloud,30,200,30);
  viewer.addPointCloud(testing_cloud,colorh,"test_data");
  // 添加检测到的中心点（红色）
  viewer.addPointCloud (colored_cloud,"centors");
  // 设置点的大小
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,10,"centors");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"test_data");
  
  // 主循环：持续刷新可视化
  while (!viewer.wasStopped ())
  {
    viewer.spin();
  }

  return (0);
}