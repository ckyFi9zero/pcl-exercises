#include <pcl/io/io.h>                                      // PCL基础IO支持
#include <pcl/io/pcd_io.h>                                  // 用于加载PCD点云文件
#include <pcl/features/integral_image_normal.h>             // 积分图像法线估计核心头文件
#include <pcl/visualization/cloud_viewer.h>                 // 可视化工具

int main()
{
    // 创建智能指针存储XYZ类型的点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // 从指定路径加载有序点云文件（如立体/深度相机数据）
    pcl::io::loadPCDFile("../table_scene_mug_stereo_textured.pcd", *cloud);
    
    // 创建输出容器，用于存储每个点的法向量
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    
    // 创建积分图像法向量估计算法对象：适用于有序点云（类似图像结构）
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    
    // 设置法向量估计方法为“3D梯度平均法”
    // AVERAGE_3D_GRADIENT: 基于局部平面拟合和梯度平均，精度较高
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    
    // 设置最大深度变化因子为0.02（2%）
    // 用于控制在计算法向量时考虑的深度差异阈值，防止跨边缘误匹配
    ne.setMaxDepthChangeFactor(0.02f);
    
    // 设置法向量平滑窗口大小为10.0
    // 数值越大，法向量越平滑；过大会丢失细节（如边缘）
    ne.setNormalSmoothingSize(10.0f);
    
    // 设置输入点云
    ne.setInputCloud(cloud);
    
    // 执行法向量计算，结果存入normals
    // 输出大小与输入一致：normals->size() == cloud->size()
    ne.compute(*normals);
    
    // 创建可视化窗口，标题为 "PCL Viewer"
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    
    // 设置背景颜色为深蓝色（R=0.0, G=0.0, B=0.5）
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    
    // 将原始点云和计算出的法向量一起添加到可视化窗口
    // 法向量将以线段形式从每个点延伸，便于观察方向
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals);

    // 进入循环，持续监听窗口事件（如旋转、缩放、关闭）
    while (!viewer.wasStopped())
    {
        // 处理一次GUI事件，保持界面响应
        viewer.spinOnce();
    }

    // 程序正常退出
    return 0;
}