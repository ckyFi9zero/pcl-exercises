#include <iostream>                             // 标准输入输出
#include <vector>                               // 动态数组，用于存储聚类索引
#include <pcl/point_types.h>                    // 点类型定义，如 PointXYZ
#include <pcl/io/pcd_io.h>                      // 读写 .pcd 文件
#include <pcl/search/search.h>                  // 通用搜索接口
#include <pcl/search/kdtree.h>                  // KdTree 加速邻域搜索
#include <pcl/features/normal_3d.h>             // 法线估计模块
#include <pcl/visualization/cloud_viewer.h>     // 简单点云可视化工具
#include <pcl/filters/passthrough.h>            // 直通滤波器（裁剪范围）
#include <pcl/segmentation/region_growing.h>    // 区域生长分割算法
#include <pcl/console/print.h>                  // PCL 日志输出宏（PCL_INFO, PCL_ERROR）
#include <pcl/console/parse.h>                  // 命令行参数解析
#include <pcl/console/time.h>                   // 时间测量支持

// 替代 Windows 平台的内存监控（windows.h + psapi.h）
#include <sys/resource.h>  // 用于获取 Linux 下内存使用（VmRSS）
#include <unistd.h>
#include <fstream>
#include <sstream>

using namespace pcl::console;  // 使用 PCL 控制台命名空间，简化 parse_argument 等调用

// Linux 下获取当前进程内存使用（单位：KB）
long getMemoryUsage() {
    std::ifstream file("/proc/self/status");   // 读取当前进程状态
    std::string line;
    while (std::getline(file, line)) {
        if (line.compare(0, 6, "VmRSS:") == 0) { // 查找 "VmRSS:" 行（物理内存占用）
            std::istringstream iss(line);
            std::string key;
            long value;
            iss >> key >> value;                 // 解析出数值
            return value;                        // 返回内存使用量（KB）
        }
    }
    return 0;
}

// 打印内存信息（Linux 版）
void PrintMemoryInfo() {
    long mem_kb = getMemoryUsage();
    std::cout << "\n--- Memory Usage (Linux) ---" << std::endl;
    std::cout << "Current RSS Memory: " << mem_kb << " KB (" 
              << (mem_kb / 1024.0) << " MB)" << std::endl;
}

int main(int argc, char** argv) {
    // 检查命令行参数是否足够
    if (argc < 2) {
        std::cout << argv[0] << " xx.pcd -kn 50 -bc 0 -fc 10.0 -nc 0 -st 30 -ct 0.05" << std::endl;
        // 提示用法：可指定法线邻域数、是否裁剪、远近裁剪距离、平滑/曲率阈值等
        return 0;
    }

    time_t start, end;                           // 用于计时
    start = time(0);
    double diff[3] = {0};                        // 存储三个阶段耗时：加载、法线、分割

    // 默认参数设置
    int KN_normal = 50;                          // 法线估计时使用的邻居点数
    bool Bool_Cuting = false;                    // 是否启用 Z 方向裁剪
    float far_cuting = 10.0f, near_cuting = 0.0f; // 裁剪范围：z ∈ [near_cuting, far_cuting]
    float SmoothnessThreshold = 30.0f;           // 平滑角阈值（单位：度）
    float CurvatureThreshold = 0.05f;            // 曲率阈值

    // 解析命令行参数（如果用户提供）
    parse_argument(argc, argv, "-kn", KN_normal); // -kn: 法线邻居数
    parse_argument(argc, argv, "-bc", Bool_Cuting); // -bc: 是否裁剪（0/1）
    parse_argument(argc, argv, "-fc", far_cuting);  // -fc: 远裁剪平面
    parse_argument(argc, argv, "-nc", near_cuting); // -nc: 近裁剪平面
    parse_argument(argc, argv, "-st", SmoothnessThreshold); // -st: 平滑角阈值（度）
    parse_argument(argc, argv, "-ct", CurvatureThreshold);  // -ct: 曲率阈值

    // 加载点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) {
        PCL_ERROR("Cloud reading failed.\n");    // 加载失败报错
        return -1;
    }
    end = time(0);
    diff[0] = difftime(end, start);              // 计算加载耗时
    PCL_INFO("Loading pcd file takes (seconds): %.2f\n", diff[0]);

    // 法线估计
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);      // 设置搜索方法为 KdTree
    normal_estimator.setInputCloud(cloud);       // 输入原始点云
    normal_estimator.setKSearch(KN_normal);      // 使用 KN_normal 个最近邻估计法线
    normal_estimator.compute(*normals);          // 计算法线并存储到 normals
    end = time(0);
    diff[1] = difftime(end, start) - diff[0];    // 计算法线估计耗时
    PCL_INFO("Estimating normal takes (seconds): %.2f\n", diff[1]);

    // 直通滤波（Z方向裁剪）
    pcl::IndicesPtr indices(new std::vector<int>); // 存储保留点的索引
    if (Bool_Cuting) {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");             // 对 z 坐标进行过滤
        pass.setFilterLimits(near_cuting, far_cuting); // 设置裁剪区间
        pass.filter(*indices);                    // 输出满足条件的点的索引
    }

    // 区域生长分割
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(50);                   // 每个聚类最小点数
    reg.setMaxClusterSize(1000000);              // 最大点数限制（防止过大）
    reg.setSearchMethod(tree);                   // 使用 KdTree 加速搜索
    reg.setNumberOfNeighbours(30);               // 每个点考虑其 30 个最近邻
    reg.setInputCloud(cloud);                    // 输入原始点云
    if (Bool_Cuting) reg.setIndices(indices);    // 如果启用了裁剪，则只在裁剪后的点上分割
    reg.setInputNormals(normals);                // 输入法线信息（区域生长依赖法线）
    reg.setSmoothnessThreshold(SmoothnessThreshold / 180.0 * M_PI); // 转换为弧度
    reg.setCurvatureThreshold(CurvatureThreshold); // 曲率阈值用于判断边界

    std::vector<pcl::PointIndices> clusters;     // 存储分割出的所有聚类
    reg.extract(clusters);                       // 执行区域生长分割
    end = time(0);
    diff[2] = difftime(end, start) - diff[0] - diff[1]; // 计算分割耗时
    PCL_INFO("Region growing takes (seconds): %.2f\n", diff[2]);

    // 输出聚类结果统计
    std::cout << "Number of clusters: " << clusters.size() << std::endl;
    if (!clusters.empty()) {
        std::cout << "First cluster has " << clusters[0].indices.size() << " points." << std::endl;
    }

    // 打印内存使用情况
    PrintMemoryInfo();

    // 可视化分割结果（彩色点云）
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    pcl::visualization::CloudViewer viewer("PCL Region Growing Segmentation");
    viewer.showCloud(colored_cloud);             // 显示着色后的点云
    while (!viewer.wasStopped()) {
        // 等待用户关闭窗口
    }

    return 0; // 成功退出
}