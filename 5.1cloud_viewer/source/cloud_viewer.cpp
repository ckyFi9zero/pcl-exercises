#include <pcl/visualization/cloud_viewer.h>   // 引入PCL可视化（CloudViewer）头文件
#include <iostream>                           // 引入标准输入输出流
#include <pcl/io/io.h>                        // 引入PCL通用IO接口
#include <pcl/io/pcd_io.h>                    // 引入PCD文件读写功能

int user_data;   // 全局变量，用于演示在回调函数与主线程间共享数据

// 【一次性回调函数】：只在可视化线程第一次刷新时执行
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(1.0, 0.5, 1.0);            // 设置背景颜色为品红（RGB：1, 0.5, 1）
    pcl::PointXYZ o;                                     // 创建空间一点
    o.x = 1.0;
    o.y = 0;
    o.z = 0;                                             // 坐标为(1,0,0)
    viewer.addSphere(o, 0.25, "sphere", 0);              // 在(1,0,0)处添加半径0.25的球体，ID为"sphere"
    std::cout << "i only run once" << std::endl;         // 控制台提示信息
}

// 【循环回调函数】：每次渲染刷新都会调用
void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;                           // 静态计数器，记录调用次数
    std::stringstream ss;                                // 字符串流，用于拼接文字
    ss << "Once per viewer loop: " << count++;           // 构造显示文本
    viewer.removeShape("text", 0);                       // 删除上一次添加的文字（ID为"text"）
    viewer.addText(ss.str(), 200, 300, "text", 0);       // 在(200,300)位置重新添加文字
    // FIXME: 此处可能存在多线程竞争条件
    user_data++;                                         // 修改全局变量
}

int main()
{
    // 创建带RGBA颜色信息的点云指针
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

    // 从上级目录加载文件到cloud
    pcl::io::loadPCDFile("../maize.pcd", *cloud);

    // 创建可视化窗口，标题为"Cloud Viewer"
    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    // 显示点云，showCloud是同步调用，会阻塞直到渲染完成
    viewer.showCloud(cloud);

    // 注册一次性回调函数：viewerOneOff将在第一次刷新时执行
    viewer.runOnVisualizationThreadOnce(viewerOneOff);

    // 注册循环回调函数：viewerPsycho将在每次刷新时执行
    viewer.runOnVisualizationThread(viewerPsycho);

    // 主循环：只要窗口未被关闭就持续运行
    while (!viewer.wasStopped())
    {
        // 可在此处添加其他实时处理逻辑
        user_data++;   // 修改全局变量
    }

    return 0;   // 程序正常退出
}