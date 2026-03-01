#include <pcl/point_types.h>                         // PCL 内置点类型
#include <pcl/io/pcd_io.h>                           // PCL 点云IO (PCD文件读写)
#include <pcl/console/time.h>                        // PCL 时间统计工具
#include <iostream>                                  // 标准输入输出
#include <ostream>                                   // 输出流
#include <pcl/filters/voxel_grid.h>                  // 体素网格滤波
#include <pcl/features/normal_3d.h>                  // 法线估计
#include <pcl/segmentation/conditional_euclidean_clustering.h> // 条件欧几里得聚类
#include <pcl/console/parse.h>                       // 命令行参数解析
#include <pcl/visualization/pcl_visualizer.h>        // 可视化工具
#include <pcl/visualization/point_cloud_color_handlers.h> // 点云着色处理器

// 定义常用类型别名
typedef pcl::PointXYZI PointTypeIO;                   // 带强度的XYZ点
typedef pcl::PointXYZINormal PointTypeFull;           // 带强度和法线的XYZ点
typedef pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;
typedef ColorHandler::ConstPtr ColorHandlerConstPtr;
using namespace pcl::console;                         // 使用PCL控制台命名空间

// 条件函数1：仅基于强度相似性
bool enforceIntensitySimilarity (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
  if (fabs ((float)point_a.intensity - (float)point_b.intensity) < 5.0f)
    return (true);
  else
    return (false);
}

// 条件函数2：基于强度或法线相似性
bool enforceCurvatureOrIntensitySimilarity (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
  // 修复：使用 Eigen::Map 正确地将法线数组映射为向量
  Eigen::Map<const Eigen::Vector3f> point_a_normal = Eigen::Map<const Eigen::Vector3f>(point_a.normal);
  Eigen::Map<const Eigen::Vector3f> point_b_normal = Eigen::Map<const Eigen::Vector3f>(point_b.normal);
  
  if (fabs ((float)point_a.intensity - (float)point_b.intensity) < 5.0f)
    return (true);
  if (fabs (point_a_normal.dot (point_b_normal)) < 0.05) // 法线夹角大（点积小）则认为是边界
    return (true);
  return (false);
}

// 条件函数3：自定义区域生长（距离相关）
bool customRegionGrowing (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
  Eigen::Map<const Eigen::Vector3f> point_a_normal = Eigen::Map<const Eigen::Vector3f>(point_a.normal);
  Eigen::Map<const Eigen::Vector3f> point_b_normal = Eigen::Map<const Eigen::Vector3f>(point_b.normal);
  
  if (squared_distance < 10000) // 距离近
  {
    if (fabs ((float)point_a.intensity - (float)point_b.intensity) < 8.0f)
      return (true);
    if (fabs (point_a_normal.dot (point_b_normal)) < 0.06) // 法线差异大
      return (true);
  }
  else // 距离远
  {
    if (fabs ((float)point_a.intensity - (float)point_b.intensity) < 3.0f) // 要求更高的强度一致性
      return (true);
  }
  return (false);
}

int main (int argc, char** argv)
{
  // 检查输入参数
  if(argc < 2)
  {
    std::cout << ".exe xx.pcd -l 40 -r 300.0 -v 1 -m 1/2/3" << std::endl;
    return 0;
  }

  bool Visual = true; // 是否启用可视化
  float Leaf = 40, Radius = 300; // 体素大小和法线估计半径
  int Method = 1; // 默认使用方法1

  // 解析命令行参数
  parse_argument (argc, argv, "-l", Leaf);
  parse_argument (argc, argv, "-r", Radius);
  parse_argument (argc, argv, "-v", Visual);
  parse_argument (argc, argv, "-m", Method);

  // 数据容器
  pcl::PointCloud<PointTypeIO>::Ptr cloud_in (new pcl::PointCloud<PointTypeIO>);
  pcl::PointCloud<PointTypeIO>::Ptr cloud_out (new pcl::PointCloud<PointTypeIO>);
  pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals (new pcl::PointCloud<PointTypeFull>);
  pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters); // 存储有效聚类
  pcl::IndicesClustersPtr small_clusters (new pcl::IndicesClusters); // 存储过小聚类
  pcl::IndicesClustersPtr large_clusters (new pcl::IndicesClusters); // 存储过大聚类
  pcl::search::KdTree<PointTypeIO>::Ptr search_tree (new pcl::search::KdTree<PointTypeIO>);
  pcl::console::TicToc tt; // 用于计时

  // 加载输入点云
  std::cerr << "Loading...\n", tt.tic ();
  pcl::io::loadPCDFile (argv[1], *cloud_in);
  std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud_in->points.size () << " points\n";

  // 体素网格下采样
  std::cerr << "Downsampling...\n", tt.tic ();
  pcl::VoxelGrid<PointTypeIO> vg;
  vg.setInputCloud (cloud_in);
  vg.setLeafSize (Leaf, Leaf, Leaf);
  vg.setDownsampleAllData (true); // 下采样所有字段（包括intensity）
  vg.filter (*cloud_out);
  std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud_out->points.size () << " points\n";

  // 计算法线
  std::cerr << "Computing normals...\n", tt.tic ();
  pcl::copyPointCloud (*cloud_out, *cloud_with_normals); // 复制点和强度
  pcl::NormalEstimation<PointTypeIO, PointTypeFull> ne;
  ne.setInputCloud (cloud_out);
  ne.setSearchMethod (search_tree);
  ne.setRadiusSearch (Radius);
  ne.compute (*cloud_with_normals); // 计算法线并填充到cloud_with_normals
  std::cerr << ">> Done: " << tt.toc () << " ms\n";

  // 设置条件欧几里得聚类
  std::cerr << "Segmenting to clusters...\n", tt.tic ();
  pcl::ConditionalEuclideanClustering<PointTypeFull> cec (true); // 初始化为true，允许自定义条件
  cec.setInputCloud (cloud_with_normals);

  // 根据参数选择不同的条件函数
  switch(Method)
  {
    case 1:
      cec.setConditionFunction (&enforceIntensitySimilarity);
      break;
    case 2:
      cec.setConditionFunction (&enforceCurvatureOrIntensitySimilarity);
      break;
    case 3:
      cec.setConditionFunction (&customRegionGrowing);
      break;
    default:
      cec.setConditionFunction (&customRegionGrowing);
      break;
  }

  cec.setClusterTolerance (500.0); // 聚类时的搜索半径
  cec.setMinClusterSize (cloud_with_normals->points.size () / 1000); // 最小聚类点数
  cec.setMaxClusterSize (cloud_with_normals->points.size () / 5); // 最大聚类点数
  cec.segment (*clusters); // 执行聚类
  cec.getRemovedClusters (small_clusters, large_clusters); // 获取被移除的聚类（太小或太大）

  std::cerr << ">> Done: " << tt.toc () << " ms\n";

  // 将聚类结果编码到输出点云的intensity字段
  // 过小的聚类标记为-2.0
  for (int i = 0; i < small_clusters->size (); ++i)
    for (int j = 0; j < (*small_clusters)[i].indices.size (); ++j)
      cloud_out->points[(*small_clusters)[i].indices[j]].intensity = -2.0;

  // 过大的聚类标记为+10.0
  for (int i = 0; i < large_clusters->size (); ++i)
    for (int j = 0; j < (*large_clusters)[i].indices.size (); ++j)
      cloud_out->points[(*large_clusters)[i].indices[j]].intensity = +10.0;

  // 有效聚类随机着色
  for (int i = 0; i < clusters->size (); ++i)
  {
    int label = rand () % 8; // 生成0-7的随机数
    for (int j = 0; j < (*clusters)[i].indices.size (); ++j)
      cloud_out->points[(*clusters)[i].indices[j]].intensity = label;
  }

  // 可视化或保存结果
  if(Visual) // 如果启用可视化
  {
    // 创建双视口可视化器
    boost::shared_ptr<pcl::visualization::PCLVisualizer> MView (new pcl::visualization::PCLVisualizer ("Conditional Euclidean Clustering"));
    int v1(0), v2(0);
    MView->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    MView->setBackgroundColor (1, 0.2, 1, v1); // 紫色
    MView->addText ("Original Cloud", 10, 10, "Before segmentation", v1);
    MView->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
    MView->setBackgroundColor (0.5, 0.5, 0.5, v2); // 灰色
    MView->addText ("Segmented Cloud", 10, 10, "After segmentation", v2);

    // 添加原始点云（视口1）
    MView->addPointCloud<pcl::PointXYZI>(cloud_in, "input", v1);
    MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input", v1);

    // 添加分割后的点云（视口2），根据intensity字段着色
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler(cloud_out, "intensity");
    MView->addPointCloud<pcl::PointXYZI>(cloud_out, color_handler, "output", v2);
    MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "output", v2);

    MView->spin(); // 启动可视化
  }
  else // 保存到文件
  {
    std::cerr << "Saving...\n", tt.tic ();
    pcl::io::savePCDFile ("output.pcd", *cloud_out);
    std::cerr << ">> Done: " << tt.toc () << " ms\n";
  }

  return (0);
}