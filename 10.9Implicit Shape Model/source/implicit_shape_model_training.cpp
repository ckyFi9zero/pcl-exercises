// 标准输入输出
#include <iostream>
// PCL PCD 文件读写
#include <pcl/io/pcd_io.h>
// 法向量估计
#include <pcl/features/normal_3d.h>
// 特征基类
#include <pcl/features/feature.h>
// 点云可视化（训练阶段未使用）
#include <pcl/visualization/cloud_viewer.h>
// FPFH 特征
#include <pcl/features/fpfh.h>
// FPFH 实现（通常不需要显式包含）
#include <pcl/features/impl/fpfh.hpp>
// 隐式形状模型 (ISM)
#include <pcl/recognition/implicit_shape_model.h>

int main (int argc, char** argv)
{
  // 检查命令行参数
  if (argc == 0) // 注意：这里应该是 argc < 3 或类似，因为 argv[0] 总是存在
  { 
    std::cout << std::endl;
    std::cout << "Usage: " << argv[0] << " class1.pcd class1_label(int) class2.pcd class2_label" << std::endl << std::endl;
    return (-1);
  }
  // 计算训练云的数量（参数数量-1，然后除以2，因为每个云对应一个.pcd和一个label）
  unsigned int number_of_training_clouds = (argc - 1) / 2;

  // 创建法向量估计器
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setRadiusSearch (25.0); // 设置法向量估计的搜索半径

  // 存储训练数据的容器
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> training_clouds; // 训练点云
  std::vector<pcl::PointCloud<pcl::Normal>::Ptr> training_normals;  // 训练法向量
  std::vector<unsigned int> training_classes;                        // 训练类别标签

  // 循环加载所有训练云
  for (unsigned int i_cloud = 0; i_cloud < number_of_training_clouds - 1; i_cloud++) // ❌ Bug: 应为 i_cloud < number_of_training_clouds
  {
    // 创建新的点云指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr tr_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
    // 从文件加载点云
    if ( pcl::io::loadPCDFile <pcl::PointXYZ> (argv[i_cloud * 2 + 1], *tr_cloud) == -1 )
      return (-1);

    // 创建法向量指针
    pcl::PointCloud<pcl::Normal>::Ptr tr_normals = (new pcl::PointCloud<pcl::Normal>)->makeShared ();
    // 设置输入并计算法向量
    normal_estimator.setInputCloud (tr_cloud);
    normal_estimator.compute (*tr_normals);

    // 将字符串标签转换为整数
    unsigned int tr_class = static_cast<unsigned int> (strtol (argv[i_cloud * 2 + 2], 0, 10));

    // 将数据存入训练集
    training_clouds.push_back (tr_cloud);
    training_normals.push_back (tr_normals);
    training_classes.push_back (tr_class);
  } // ❌ Bug: 最后一个训练云没有被处理

  // 创建 FPFH 特征估计器
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> >::Ptr fpfh
    (new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> >);
  fpfh->setRadiusSearch (30.0); // 设置 FPFH 的搜索半径
  // 创建特征估计器的基类指针，指向 FPFH 实例
  pcl::Feature< pcl::PointXYZ, pcl::Histogram<153> >::Ptr feature_estimator(fpfh);
 
  // 创建 ISM 估计器
  pcl::ism::ImplicitShapeModelEstimation<153, pcl::PointXYZ, pcl::Normal> ism;
  ism.setFeatureEstimator(feature_estimator); // 设置特征估计器
  ism.setTrainingClouds (training_clouds);    // 设置训练点云
  ism.setTrainingNormals (training_normals);  // 设置训练法向量
  ism.setTrainingClasses (training_classes);  // 设置训练类别
  ism.setSamplingSize (2.0f);                 // 设置采样大小（用于投票空间）

  // 创建 ISM 模型指针
  pcl::ism::ImplicitShapeModelEstimation<153, pcl::PointXYZ, pcl::Normal>::ISMModelPtr model = 
    boost::shared_ptr<pcl::features::ISMModel> (new pcl::features::ISMModel);
  // 训练 ISM 模型
  ism.trainISM (model);

  // 保存训练好的模型到文件
  std::string file ("trained_ism_model.txt");
  model->saveModelToFile (file);

  // 输出提示信息
  std::cout << "trained_ism_model.txt is the output of training stage. You can use the trained_ism_model.txt in the classification stage" << std::endl << std::endl;

  return (0);
}