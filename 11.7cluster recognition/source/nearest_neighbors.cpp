#include <pcl/point_types.h>                     // PCL点类型定义
#include <pcl/point_cloud.h>                     // PCL点云数据结构
#include <pcl/common/common.h>                   // PCL通用算法，如计算质心
#include <pcl/common/transforms.h>               // PCL点云变换
#include <pcl/visualization/pcl_visualizer.h>    // PCL可视化工具
#include <pcl/console/parse.h>                   // PCL命令行解析
#include <pcl/console/print.h>                   // PCL控制台输出
#include <pcl/io/pcd_io.h>                       // PCL的PCD文件读写
#include <iostream>                              // C++标准输入输出
#include <flann/flann.h>                         // FLANN库主头文件
#include <flann/io/hdf5.h>                       // FLANN的HDF5文件读写
#include <boost/filesystem.hpp>                  // Boost文件系统库
// 修复：添加缺失的头文件
#include <string>                                // std::string
#include <vector>                                // std::vector

typedef std::pair<std::string, std::vector<float> > vfh_model; // 定义VFH模型类型，包含文件路径和特征向量

/** \brief Loads an n-D histogram file as a VFH signature
  * \param path the input file name
  * \param vfh the resultant VFH model
  */
bool
loadHist (const boost::filesystem::path &path, vfh_model &vfh)
{
  int vfh_idx;
  // Load the file as a PCD
  try
  {
    pcl::PCLPointCloud2 cloud;                   // 用于存储PCD文件头信息
    int version;                                 // PCD文件版本
    Eigen::Vector4f origin;                      // PCD文件原点
    Eigen::Quaternionf orientation;              // PCD文件朝向
    pcl::PCDReader r;                            // PCD文件读取器
    int type; unsigned int idx;                  // PCD文件类型和索引
    r.readHeader (path.string (), cloud, origin, orientation, version, type, idx); // 读取PCD文件头

    vfh_idx = pcl::getFieldIndex (cloud, "vfh"); // 查找"vfh"字段的索引
    if (vfh_idx == -1)                           // 如果没有找到"vfh"字段
      return (false);                            // 返回false
    if ((int)cloud.width * cloud.height != 1)    // VFH特征文件应为1x1
      return (false);                            // 如果不是，返回false
  }
  catch (pcl::InvalidConversionException &e)     // 捕获异常（修复：添加引用）
  {
    return (false);                              // 发生异常，返回false
  }
  // Treat the VFH signature as a single Point Cloud
  pcl::PointCloud <pcl::VFHSignature308> point;  // 声明一个VFH点云
  pcl::io::loadPCDFile (path.string (), point);  // 加载完整的PCD文件
  vfh.second.resize (308);                       // 为特征向量分配空间

  std::vector <pcl::PCLPointField> fields;       // 存储点云字段信息
  pcl::getFieldIndex (point, "vfh", fields);     // 获取"vfh"字段信息

  for (size_t i = 0; i < fields[vfh_idx].count; ++i) // 遍历308个特征值
  {
    vfh.second[i] = point.points[0].histogram[i]; // 复制特征值
  }
  vfh.first = path.string ();                    // 保存文件路径
  return (true);                                 // 加载成功
}

/** \brief Search for the closest k neighbors
  * \param index the tree
  * \param model the query model
  * \param k the number of neighbors to search for
  * \param indices the resultant neighbor indices
  * \param distances the resultant neighbor distances
  */
inline void
nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model, 
                int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances)
{
  // Query point
  flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size ()); // 创建查询点矩阵
  memcpy (&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float)); // 复制特征向量到矩阵
  indices = flann::Matrix<int>(new int[k], 1, k); // 为索引分配内存
  distances = flann::Matrix<float>(new float[k], 1, k); // 为距离分配内存
  index.knnSearch (p, indices, distances, k, flann::SearchParams (512)); // 执行KNN搜索
  delete[] p.ptr ();                             // 释放查询点内存
}

/** \brief Load the list of file model names from an ASCII file
  * \param models the resultant list of model name
  * \param filename the input file name
  */
bool
loadFileList (std::vector<vfh_model> &models, const std::string &filename)
{
  std::ifstream fs;                              // 创建输入文件流
  fs.open (filename.c_str ());                   // 打开文件
  if (!fs.is_open () || fs.fail ())              // 如果打开失败
    return (false);                              // 返回false
  std::string line;                              // 存储每一行
  while (!fs.eof ())                             // 读取到文件末尾
  {
    getline (fs, line);                          // 读取一行
    if (line.empty ())                           // 如果是空行
      continue;                                  // 跳过
    vfh_model m;                                 // 创建一个vfh_model
    m.first = line;                              // 设置文件路径
    models.push_back (m);                        // 添加到models向量
  }
  fs.close ();                                   // 关闭文件
  return (true);                                 // 加载成功
}

int
main (int argc, char** argv)
{
  int k = 6;                                     // 默认k值
  double thresh = DBL_MAX;                       // 默认阈值（禁用）
  if (argc < 2)                                  // 检查参数数量
  {
    pcl::console::print_error ("Need at least one parameter! Syntax is: %s <query_vfh_model.pcd> [options] {kdtree.idx} {training_data.h5} {training_data.list}\n", argv[0]); // 打印错误信息
    pcl::console::print_info ("    where [options] are:  -k      = number of nearest neighbors to search for in the tree (default: "); 
    pcl::console::print_value ("%d", k); pcl::console::print_info (")\n");
    pcl::console::print_info ("                          -thresh = maximum distance threshold for a model to be considered VALID (default: "); 
    pcl::console::print_value ("%f", thresh); pcl::console::print_info (")\n");
    return (-1);                                 // 返回错误码
  }

  std::string extension (".pcd");                // 文件扩展名
  std::transform (extension.begin (), extension.end (), extension.begin (), ::tolower); // 转换为小写

  // Load the test histogram
  std::vector<int> pcd_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd"); // 解析命令行中的PCD文件
  if (pcd_indices.empty())                       // 如果没有找到PCD文件
  {
    pcl::console::print_error("No PCD file found in the command line arguments.\n"); // 打印错误
    return -1;                                   // 返回错误
  }

  vfh_model histogram;                           // 存储查询的VFH模型
  if (!loadHist (argv[pcd_indices.at (0)], histogram)) // 加载查询的VFH特征
  {
    pcl::console::print_error ("Cannot load test file %s\n", argv[pcd_indices.at (0)]); // 加载失败
    return (-1);                                 // 返回错误
  }
  pcl::console::parse_argument (argc, argv, "-thresh", thresh); // 解析-thresh参数
  // Search for the k closest matches
  pcl::console::parse_argument (argc, argv, "-k", k); // 解析-k参数
  pcl::console::print_highlight ("Using "); pcl::console::print_value ("%d", k); pcl::console::print_info (" nearest neighbors.\n"); // 打印信息

  std::string kdtree_idx_file_name = "kdtree.idx"; // 索引文件名
  std::string training_data_h5_file_name = "training_data.h5"; // HDF5数据文件名
  std::string training_data_list_file_name = "training_data.list"; // 文件列表名
  std::vector<vfh_model> models;                 // 存储训练模型
  flann::Matrix<int> k_indices;                  // 存储搜索结果索引
  flann::Matrix<float> k_distances;              // 存储搜索结果距离
  flann::Matrix<float> data;                     // 存储从HDF5加载的特征数据

  // Check if the data has already been saved to disk
  if (!boost::filesystem::exists (training_data_h5_file_name) || !boost::filesystem::exists (training_data_list_file_name)) // 检查训练数据文件是否存在
  {
    pcl::console::print_error ("Could not find training data models files %s and %s!\n", 
        training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ()); // 打印错误
    return (-1);                                 // 返回错误
  }
  else
  {
    if (!loadFileList (models, training_data_list_file_name)) // 加载文件列表
    {
      pcl::console::print_error("Failed to load file list from %s\n", training_data_list_file_name.c_str()); // 加载失败
      return -1;                                 // 返回错误
    }
    flann::load_from_file (data, training_data_h5_file_name, "training_data"); // 从HDF5加载特征数据
    pcl::console::print_highlight ("Training data found. Loaded %d VFH models from %s/%s.\n", 
        (int)data.rows, training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ()); // 打印信息
  }

  // Check if the tree index has already been saved to disk
  if (!boost::filesystem::exists (kdtree_idx_file_name)) // 检查索引文件是否存在
  {
    pcl::console::print_error ("Could not find kd-tree index in file %s!\n", kdtree_idx_file_name.c_str ()); // 打印错误
    return (-1);                                 // 返回错误
  }
  else
  {
    // 修复：使用 SavedIndexParams 加载，不要调用 buildIndex()
    flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams (kdtree_idx_file_name)); // 从文件加载索引
    // index.buildIndex (); // ? 删除这行，buildIndex 与 SavedIndexParams 冲突
    nearestKSearch (index, histogram, k, k_indices, k_distances); // 执行KNN搜索
  }

  // Output the results on screen
  pcl::console::print_highlight ("The closest %d neighbors for %s are:\n", k, argv[pcd_indices[0]]); // 打印结果标题
  for (int i = 0; i < k; ++i)                    // 遍历k个最近邻
  {
    // 修复：检查索引有效性
    if (k_indices[0][i] < 0 || k_indices[0][i] >= static_cast<int>(models.size())) // 检查索引是否越界
    {
      pcl::console::print_error("Invalid neighbor index: %d\n", k_indices[0][i]); // 打印错误
      continue;                                  // 跳过
    }
    pcl::console::print_info ("    %d - %s (%d) with a distance of: %f\n", 
        i, models.at (k_indices[0][i]).first.c_str (), k_indices[0][i], k_distances[0][i]); // 打印每个邻居的信息
  }

  // Load the results
  pcl::visualization::PCLVisualizer p (argc, argv, "VFH Cluster Classifier"); // 创建可视化窗口
  int y_s = (int)floor (sqrt ((double)k));       // 计算视口行数
  int x_s = y_s + (int)ceil ((k / (double)y_s) - y_s); // 计算视口列数
  double x_step = (double)(1.0 / (double)x_s);   // 计算每个视口的宽度
  double y_step = (double)(1.0 / (double)y_s);   // 计算每个视口的高度
  pcl::console::print_highlight ("Preparing to load "); 
  pcl::console::print_value ("%d", k); 
  pcl::console::print_info (" files ("); 
  pcl::console::print_value ("%d", x_s);    
  pcl::console::print_info ("x"); 
  pcl::console::print_value ("%d", y_s); 
  pcl::console::print_info (" / ");
  pcl::console::print_value ("%f", x_step); 
  pcl::console::print_info ("x"); 
  pcl::console::print_value ("%f", y_step); 
  pcl::console::print_info (")\n");

  int viewport = 0, l = 0, m = 0;                // 视口计数器和行列计数器
  for (int i = 0; i < k; ++i)                    // 遍历k个最近邻
  {
    // 修复：检查索引有效性
    if (k_indices[0][i] < 0 || k_indices[0][i] >= static_cast<int>(models.size())) // 检查索引是否有效
    {
      pcl::console::print_error("Skipping invalid neighbor index: %d\n", k_indices[0][i]); // 打印错误
      continue;                                  // 跳过
    }

    std::string cloud_name = models.at (k_indices[0][i]).first; // 获取模型文件路径
    // 修复：安全地移除 "_vfh.pcd" 后缀
    size_t pos = cloud_name.rfind("_vfh.pcd");   // 查找"_vfh.pcd"
    if (pos != std::string::npos)                // 如果找到
    {
      cloud_name.replace(pos, 8, ".pcd");        // 替换为".pcd"
    }
    else
    {
      // 如果没有找到，尝试移除 "_vfh"
      pos = cloud_name.rfind("_vfh");            // 查找"_vfh"
      if (pos != std::string::npos)              // 如果找到
      {
        cloud_name.replace(pos, 4, "");          // 移除"_vfh"
      }
      // 如果还是找不到，保留原名
      pcl::console::print_warn("Filename does not have '_vfh' suffix: %s\n", cloud_name.c_str()); // 打印警告
    }

    p.createViewPort (l * x_step, m * y_step, (l + 1) * x_step, (m + 1) * y_step, viewport); // 创建视口
    l++;
    if (l >= x_s)
    {
      l = 0;
      m++;
    }

    pcl::PCLPointCloud2 cloud;                   // 存储加载的PCD数据
    pcl::console::print_highlight ("Loading "); pcl::console::print_value ("%s ", cloud_name.c_str ()); // 打印加载信息
    if (pcl::io::loadPCDFile (cloud_name, cloud) == -1) // 加载PCD文件
    {
      pcl::console::print_error("Failed to load %s\n", cloud_name.c_str()); // 加载失败
      viewport++;                                // 递增视口计数器
      continue;                                  // 跳过
    }

    // Convert from blob to PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;    // 存储转换后的点云
    pcl::fromPCLPointCloud2 (cloud, cloud_xyz);  // 转换数据类型
    if (cloud_xyz.points.empty())                // 如果点云为空
    {
      pcl::console::print_error("Loaded point cloud is empty: %s\n", cloud_name.c_str()); // 打印错误
      viewport++;
      continue;
    }
    pcl::console::print_info ("[done, %zu points]\n", cloud_xyz.points.size()); // 打印加载成功
    pcl::console::print_info ("Available dimensions: %s\n", pcl::getFieldsList (cloud).c_str ()); // 打印字段信息

    // Demean the cloud
    Eigen::Vector4f centroid;                    // 存储质心
    pcl::compute3DCentroid (cloud_xyz, centroid); // 计算质心
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_demean (new pcl::PointCloud<pcl::PointXYZ>); // 创建去中心化点云
    pcl::demeanPointCloud<pcl::PointXYZ> (cloud_xyz, centroid, *cloud_xyz_demean); // 去中心化

    // 修复：使用基于viewport的唯一ID
    std::stringstream cloud_id;                  // 创建ID
    cloud_id << "cloud_" << viewport;            // ID为 "cloud_0", "cloud_1", ...
    p.addPointCloud (cloud_xyz_demean, cloud_id.str(), viewport); // 添加点云到视口

    // Check if the model found is within our inlier tolerance
    std::stringstream ss;                        // 创建字符串流
    ss << std::fixed << std::setprecision(6) << k_distances[0][i]; // 格式化距离
    std::stringstream score_id;                  // 创建分数文本ID
    score_id << "score_" << viewport;            // ID为 "score_0", "score_1", ...
    if (k_distances[0][i] > thresh)              // 如果距离大于阈值
    {
      p.addText (ss.str (), 20, 30, 1, 0, 0, score_id.str (), viewport);  // 添加红色文本
      // Create a red line（创建一条红线）
      pcl::PointXYZ min_p, max_p;                // 存储点云的最小最大点
      pcl::getMinMax3D (*cloud_xyz_demean, min_p, max_p); // 获取边界
      std::stringstream line_name;               // 创建线ID
      line_name << "line_" << viewport;          // ID为 "line_0", "line_1", ...
      p.addLine (min_p, max_p, 1, 0, 0, line_name.str (), viewport); // 添加红线
      p.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, line_name.str (), viewport); // 设置线宽
    }
    else
    {
      p.addText (ss.str (), 20, 30, 0, 1, 0, score_id.str (), viewport); // 添加绿色文本
    }
    // Increase the font size for the score*
    p.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 18, score_id.str (), viewport); // 设置字体大小

    // Add the cluster name（添加聚类名称）
    // 修复：使用基于viewport的唯一ID，并只显示文件名
    boost::filesystem::path pth(cloud_name);     // 解析文件路径
    std::string filename = pth.filename().string(); // 获取文件名
    std::stringstream label_id;                  // 创建标签ID
    label_id << "label_" << viewport;            // ID为 "label_0", "label_1", ...
    p.addText (filename, 20, 10, label_id.str (), viewport); // 添加文件名标签

    viewport++; // 重要：在循环末尾递增viewport
  }

  // Add coordinate systems to all viewports（给所有的窗口添加坐标系统）
  p.addCoordinateSystem (0.1);                   // 添加坐标系
  p.spin ();                                     // 进入可视化循环
  return (0);                                    // 程序成功
}