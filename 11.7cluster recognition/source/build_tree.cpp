#include <pcl/point_types.h>                     // PCL点类型定义，如 pcl::PointXYZ, pcl::VFHSignature308
#include <pcl/point_cloud.h>                     // PCL点云数据结构 pcl::PointCloud
#include <pcl/console/parse.h>                   // PCL命令行解析工具
#include <pcl/console/print.h>                   // PCL控制台输出工具，如高亮、打印
#include <pcl/io/pcd_io.h>                       // PCL的PCD文件读写功能
#include <boost/filesystem.hpp>                  // Boost文件系统库，用于遍历目录和文件
#include <flann/flann.h>                         // FLANN库主头文件，用于最近邻搜索
#include <flann/io/hdf5.h>                       // FLANN的HDF5文件读写功能
#include <fstream>                               // C++标准文件流，用于写入文本文件

typedef std::pair<std::string, std::vector<float> > vfh_model; // 定义一个别名，表示一个VFH模型，包含文件路径和308维特征向量

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
    pcl::PCLPointCloud2 cloud;                   // 用于存储PCD文件头信息的通用点云数据结构
    int version;                                 // PCD文件版本
    Eigen::Vector4f origin;                      // PCD文件的原点
    Eigen::Quaternionf orientation;              // PCD文件的朝向
    pcl::PCDReader r;                            // PCD文件读取器
    int type; unsigned int idx;                  // 用于存储PCD文件的类型和索引
    r.readHeader (path.string (), cloud, origin, orientation, version, type, idx); // 只读取PCD文件头，不加载数据

    vfh_idx = pcl::getFieldIndex (cloud, "vfh"); // 在点云字段中查找名为 "vfh" 的字段索引
    if (vfh_idx == -1)                           // 如果没有找到 "vfh" 字段
      return (false);                            // 返回false，加载失败
    if ((int)cloud.width * cloud.height != 1)    // VFH特征文件应为1x1的点云（单个点）
      return (false);                            // 如果不是，返回false
  }
  catch (pcl::InvalidConversionException e)      // 捕获PCD读取异常
  {
    return (false);                              // 发生异常，返回false
  }

  // Treat the VFH signature as a single Point Cloud
  pcl::PointCloud <pcl::VFHSignature308> point;  // 声明一个VFH特征点云，它只包含一个点，该点有308维直方图
  pcl::io::loadPCDFile (path.string (), point);  // 加载完整的PCD文件到VFH点云中
  vfh.second.resize (308);                       // 为vfh模型的特征向量分配308个float的空间

  std::vector <pcl::PCLPointField> fields;       // 存储点云字段信息的向量
  pcl::getFieldIndex (point, "vfh", fields);     // 获取 "vfh" 字段的详细信息

  for (size_t i = 0; i < fields[vfh_idx].count; ++i) // 遍历 "vfh" 字段的所有值（308个）
  {
    vfh.second[i] = point.points[0].histogram[i]; // 将加载的VFH特征值复制到vfh模型中
  }
  vfh.first = path.string ();                    // 将文件路径保存到vfh模型中
  return (true);                                 // 加载成功，返回true
}

/** \brief Load a set of VFH features that will act as the model (training data)
  * \param argc the number of arguments (pass from main ())
  * \param argv the actual command line arguments (pass from main ())
  * \param extension the file extension containing the VFH features
  * \param models the resultant vector of histogram models
  */
void
loadFeatureModels (const boost::filesystem::path &base_dir, const std::string &extension, 
                   std::vector<vfh_model> &models)
{
  if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir)) // 检查基础目录是否存在且为目录
    return;                                      // 如果不存在或不是目录，直接返回

  for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it) // 遍历目录下的所有条目
  {
    if (boost::filesystem::is_directory (it->status ())) // 如果当前条目是子目录
    {
      std::stringstream ss;                      // 创建字符串流
      ss << it->path ();                         // 将子目录路径转换为字符串
      pcl::console::print_highlight ("Loading %s (%lu models loaded so far).\n", ss.str ().c_str (), (unsigned long)models.size ()); // 打印正在加载的目录和已加载模型数量
      loadFeatureModels (it->path (), extension, models); // 递归调用自身，加载子目录中的模型
    }
    if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension) // 如果是普通文件且扩展名匹配
    {
      vfh_model m;                               // 创建一个新的vfh_model
      if (loadHist (base_dir / it->path ().filename (), m)) // 调用loadHist加载该文件
        models.push_back (m);                    // 如果加载成功，将模型添加到models向量中
    }
  }
}

int
main (int argc, char** argv)
{
  if (argc < 2)                                  // 检查命令行参数数量
  {
    PCL_ERROR ("Need at least two parameters! Syntax is: %s [model_directory] [options]\n", argv[0]); // 如果参数不足，打印错误信息
    return (-1);                                 // 返回错误码
  }

  std::string extension (".pcd");                // 定义要查找的文件扩展名
  transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower); // 将扩展名转换为小写

  std::string kdtree_idx_file_name = "kdtree.idx"; // 定义FLANN索引文件的保存路径
  std::string training_data_h5_file_name = "training_data.h5"; // 定义HDF5训练数据文件的保存路径
  std::string training_data_list_file_name = "training_data.list"; // 定义模型文件列表的保存路径

  std::vector<vfh_model> models;                 // 声明一个向量，用于存储所有加载的VFH模型

  // Load the model histograms
  loadFeatureModels (argv[1], extension, models); // 调用loadFeatureModels，从命令行指定的目录中递归加载所有.pcd文件
  pcl::console::print_highlight ("Loaded %d VFH models. Creating training data %s/%s.\n", 
      (int)models.size (), training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ()); // 打印加载完成信息

  // Convert data into FLANN format
  flann::Matrix<float> data (new float[models.size () * models[0].second.size ()], models.size (), models[0].second.size ()); // 创建一个FLANN矩阵，用于存储所有模型的特征向量

  for (size_t i = 0; i < data.rows; ++i)         // 遍历每一行（每个模型）
    for (size_t j = 0; j < data.cols; ++j)       // 遍历每一列（特征向量的每个维度）
      data[i][j] = models[i].second[j];          // 将models向量中的特征值复制到data矩阵中

  // Save data to disk (list of models)
  flann::save_to_file (data, training_data_h5_file_name, "training_data"); // 将data矩阵保存为HDF5文件
  std::ofstream fs;                              // 创建一个输出文件流
  fs.open (training_data_list_file_name.c_str ()); // 打开文本文件
  for (size_t i = 0; i < models.size (); ++i)    // 遍历所有模型
    fs << models[i].first << "\n";               // 将每个模型的文件路径写入文本文件
  fs.close ();                                   // 关闭文件流

  // Build the tree index and save it to disk
  pcl::console::print_error ("Building the kdtree index (%s) for %d elements...\n", kdtree_idx_file_name.c_str (), (int)data.rows); // 打印正在构建索引的信息
  flann::Index<flann::ChiSquareDistance<float> > index (data, flann::LinearIndexParams ()); // 创建一个FLANN索引对象，使用线性搜索（此处有误，应为KDTree等）
  //flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4)); // 正确的创建方式：使用KD树，4个树
  index.buildIndex ();                           // 构建FLANN搜索索引
  index.save (kdtree_idx_file_name);             // 将构建好的索引保存到文件
  delete[] data.ptr ();                          // 释放data矩阵分配的内存

  return (0);                                    // 程序成功执行，返回0
}