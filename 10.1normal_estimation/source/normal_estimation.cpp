#include <pcl/io/io.h>                      // PCL基本输入输出功能
#include <pcl/io/pcd_io.h>                  // 用于加载和保存PCD格式点云文件
#include <pcl/features/integral_image_normal.h>  // 积分图像法线估计（本例未使用，可选包含）
#include <pcl/visualization/cloud_viewer.h> // 提供简单的3D可视化工具
#include <pcl/point_types.h>                // 定义点类型，如PointXYZ、Normal
#include <pcl/features/normal_3d.h>         // 核心头文件：3D法向量估计类

int main()
{
    // 创建智能指针，存储XYZ类型的点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // 从指定路径加载PCD点云文件到cloud中
    pcl::io::loadPCDFile("../table_scene_lms400.pcd", *cloud);
    
    // 创建法向量估计算法对象：输入为PointXYZ，输出为Normal
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    
    // 设置输入点云，作为法向量计算的数据源
    ne.setInputCloud(cloud);
    
    // 创建KD-Tree搜索对象，用于加速邻近点查找
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    
    // 将KD-Tree设置为法向量估计器的搜索方法
    ne.setSearchMethod(tree);
    
    // 创建输出容器，用于存储每个点的法向量（normal_x, normal_y, normal_z, curvature）
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    
    // 设置邻域搜索半径为0.03米（3厘米）
    // 即对每个点，查找其周围3cm范围内的所有邻居用于法向量计算
    ne.setRadiusSearch(0.03);
    
    // 执行法向量计算，结果存储在cloud_normals中
    // 输出点数与输入点数相同：cloud_normals->size() == cloud->size()
    ne.compute(*cloud_normals);
    
    // 创建可视化窗口，标题为"PCL Viewer"
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    
    // 设置可视化背景颜色为黑色（RGB: 0,0,0）
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    
    // 将原始点云和对应的法向量添加到可视化窗口
    // 法向量将以线段形式从每个点延伸出来，便于观察方向
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals);

    // 进入循环，持续监听窗口事件（如鼠标旋转、缩放、关闭）
    while (!viewer.wasStopped())
    {
        // 处理一次GUI事件，保持窗口响应
        viewer.spinOnce();
    }

    // 程序正常退出
    return 0;
}