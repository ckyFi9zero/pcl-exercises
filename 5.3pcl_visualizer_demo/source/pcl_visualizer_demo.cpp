/* \author Geoffrey Biggs */      // 原作者注释

// ========== 头文件 ==========
#include <iostream>                                       // 标准输入输出
#include <boost/thread/thread.hpp>                        // Boost 线程，用于 viewer->spinOnce 延时
#include <pcl/common/common_headers.h>                    // PCL 通用头文件（重复一次，保留）
#include <pcl/common/common_headers.h>                    // PCL 通用头文件（同上）
#include <pcl/features/normal_3d.h>                       // 法线估计
#include <pcl/io/pcd_io.h>                                // PCD 文件读写
#include <pcl/visualization/pcl_visualizer.h>             // PCL 高级可视化器
#include <pcl/console/parse.h>                            // 命令行解析

// ------------------------------------------------------------
// 帮助函数：打印程序用法
// ------------------------------------------------------------
void printUsage(const char* progName)
{
  std::cout << "\n\nUsage: " << progName << " [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"               // 帮助
            << "-s           Simple visualisation example\n"   // 简单可视化
            << "-r           RGB colour visualisation example\n" // RGB 颜色
            << "-c           Custom colour visualisation example\n"// 自定义颜色
            << "-n           Normals visualisation example\n"       // 法线
            << "-a           Shapes visualisation example\n"        // 几何形体
            << "-v           Viewports example\n"                   // 多视口
            << "-i           Interaction Customization example\n"   // 交互定制
            << "\n\n";
}

// ------------------------------------------------------------
// 1) 简单可视化：只显示点云
// ------------------------------------------------------------
boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // 新建可视化窗口，标题“3D Viewer”
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);                                    // 黑色背景
  viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");            // 添加点云
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");  // 点大小=1
  viewer->addCoordinateSystem(1.0);                                       // 显示坐标轴
  viewer->initCameraParameters();                                         // 初始化相机
  return viewer;
}

// ------------------------------------------------------------
// 2) RGB 颜色可视化：使用点云自带 RGB 字段
// ------------------------------------------------------------
boost::shared_ptr<pcl::visualization::PCLVisualizer>
rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);                                    // 黑色背景
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");    // 使用 RGB 着色
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");  // 点大小=3
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  return viewer;
}

// ------------------------------------------------------------
// 3) 自定义颜色可视化：整片云指定单一颜色
// ------------------------------------------------------------
boost::shared_ptr<pcl::visualization::PCLVisualizer>
customColourVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  // 用自定义颜色（绿色 0,255,0）
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(
      cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  return viewer;
}

// ------------------------------------------------------------
// 4) 法线可视化：同时显示点云和法线
// ------------------------------------------------------------
boost::shared_ptr<pcl::visualization::PCLVisualizer>
normalsVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
           pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  // 添加法线：每10个点显示1个，长度0.05
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(
      cloud, normals, 10, 0.05, "normals");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  return viewer;
}

// ------------------------------------------------------------
// 5) 几何形体可视化：线、球、平面、圆锥
// ------------------------------------------------------------
boost::shared_ptr<pcl::visualization::PCLVisualizer>
shapesVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  // 连接第1个点与最后一个点的线段
  viewer->addLine<pcl::PointXYZRGB>(cloud->points[0],
                                    cloud->points[cloud->size() - 1], "line");

  // 在第一个点处画半径0.2的球体
  viewer->addSphere(cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

  // 添加平面：ax+by+cz+d=0  -> (0,0,1,0) 即 z=0
  pcl::ModelCoefficients coeffs;
  coeffs.values.push_back(0.0);
  coeffs.values.push_back(0.0);
  coeffs.values.push_back(1.0);
  coeffs.values.push_back(0.0);
  viewer->addPlane(coeffs, "plane");

  // 添加圆锥：圆锥参数 (0.3,0.3,0, 0,1,0, 5) 表示…
  coeffs.values.clear();
  coeffs.values.push_back(0.3);
  coeffs.values.push_back(0.3);
  coeffs.values.push_back(0.0);
  coeffs.values.push_back(0.0);
  coeffs.values.push_back(1.0);
  coeffs.values.push_back(0.0);
  coeffs.values.push_back(5.0);
  viewer->addCone(coeffs, "cone");

  return viewer;
}

// ------------------------------------------------------------
// 6) 多视口可视化：左右两个视口，分别显示不同法线
// ------------------------------------------------------------
boost::shared_ptr<pcl::visualization::PCLVisualizer>
viewportsVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
             pcl::PointCloud<pcl::Normal>::ConstPtr normals1,
             pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->initCameraParameters();

  // 创建左侧视口 (xmin,ymin,xmax,ymax) = (0,0,0.5,1)
  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor(0, 0, 0, v1);
  viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);

  // 在 v1 添加彩色点云和法线1
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud1", v1);
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(
      cloud, normals1, 10, 0.05, "normals1", v1);

  // 创建右侧视口 (0.5,0,1,1)
  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
  viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);

  // 在 v2 添加绿色点云和法线2
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(
      cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, single_color, "sample cloud2", v2);
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(
      cloud, normals2, 10, 0.05, "normals2", v2);

  // 统一设置点大小
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
  viewer->addCoordinateSystem(1.0);
  return viewer;
}

// ------------------------------------------------------------
// 7) 交互定制可视化：键盘/鼠标回调示例
// ------------------------------------------------------------
unsigned int text_id = 0;   // 用于生成文本 ID

// 键盘回调：按 'r' 删除全部文字
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
                           void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer =
      *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer>*>(viewer_void);
  if (event.getKeySym() == "r" && event.keyDown())
  {
    std::cout << "r was pressed => removing all text" << std::endl;
    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf(str, "text#%03d", i);
      viewer->removeShape(str);
    }
    text_id = 0;
  }
}

// 鼠标回调：左键点击在窗口上生成文字
void mouseEventOccurred(const pcl::visualization::MouseEvent& event,
                        void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer =
      *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer>*>(viewer_void);
  if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
      event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX() << ", "
              << event.getY() << ")" << std::endl;
    char str[512];
    sprintf(str, "text#%03d", text_id++);
    viewer->addText("clicked here", event.getX(), event.getY(), str);
  }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer>
interactionCustomizationVis()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(1.0);
  // 注册键盘和鼠标回调
  viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
  viewer->registerMouseCallback(mouseEventOccurred, (void*)&viewer);
  return viewer;
}

// ========== 主函数 ==========
int main(int argc, char** argv)
{
  // 1. 解析命令行：若有 -h 直接打印帮助
  if (pcl::console::find_argument(argc, argv, "-h") >= 0)
  {
    printUsage(argv[0]);
    return 0;
  }

  // 2. 根据输入选项设置布尔变量
  bool simple(false), rgb(false), custom_c(false), normals(false),
      shapes(false), viewports(false), interaction_customization(false);

  if (pcl::console::find_argument(argc, argv, "-s") >= 0)
  {
    simple = true;
    std::cout << "Simple visualisation example\n";
  }
  else if (pcl::console::find_argument(argc, argv, "-c") >= 0)
  {
    custom_c = true;
    std::cout << "Custom colour visualisation example\n";
  }
  else if (pcl::console::find_argument(argc, argv, "-r") >= 0)
  {
    rgb = true;
    std::cout << "RGB colour visualisation example\n";
  }
  else if (pcl::console::find_argument(argc, argv, "-n") >= 0)
  {
    normals = true;
    std::cout << "Normals visualisation example\n";
  }
  else if (pcl::console::find_argument(argc, argv, "-a") >= 0)
  {
    shapes = true;
    std::cout << "Shapes visualisation example\n";
  }
  else if (pcl::console::find_argument(argc, argv, "-v") >= 0)
  {
    viewports = true;
    std::cout << "Viewports example\n";
  }
  else if (pcl::console::find_argument(argc, argv, "-i") >= 0)
  {
    interaction_customization = true;
    std::cout << "Interaction Customization example\n";
  }
  else
  {
    printUsage(argv[0]);
    return 0;
  }

  // 3. 生成示例点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::cout << "Genarating example point clouds.\n\n";

  uint8_t r(255), g(15), b(15); // 初始颜色
  for (float z(-1.0); z <= 1.0; z += 0.05)           // 沿 z 轴生成多层
  {
    for (float angle(0.0); angle <= 360.0; angle += 5.0) // 每层画一圈
    {
      pcl::PointXYZ basic_point;
      basic_point.x = 0.5 * cosf(pcl::deg2rad(angle));
      basic_point.y = sinf(pcl::deg2rad(angle));
      basic_point.z = z;
      basic_cloud_ptr->points.push_back(basic_point);

      pcl::PointXYZRGB point;
      point.x = basic_point.x;
      point.y = basic_point.y;
      point.z = basic_point.z;
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                      static_cast<uint32_t>(g) << 8 |
                      static_cast<uint32_t>(b));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back(point);
    }
    // 颜色渐变
    if (z < 0.0)
    {
      r -= 12;
      g += 12;
    }
    else
    {
      g -= 12;
      b += 12;
    }
  }

  basic_cloud_ptr->width = static_cast<int>(basic_cloud_ptr->points.size());
  basic_cloud_ptr->height = 1;
  point_cloud_ptr->width = static_cast<int>(point_cloud_ptr->points.size());
  point_cloud_ptr->height = 1;

  // 4. 计算法线（两种半径）
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud(point_cloud_ptr);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  ne.setSearchMethod(tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch(0.05);
  ne.compute(*cloud_normals1);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch(0.1);
  ne.compute(*cloud_normals2);

  // 5. 根据命令行选项创建对应可视化器
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  if (simple)
    viewer = simpleVis(basic_cloud_ptr);
  else if (rgb)
    viewer = rgbVis(point_cloud_ptr);
  else if (custom_c)
    viewer = customColourVis(basic_cloud_ptr);
  else if (normals)
    viewer = normalsVis(point_cloud_ptr, cloud_normals2);
  else if (shapes)
    viewer = shapesVis(point_cloud_ptr);
  else if (viewports)
    viewer = viewportsVis(point_cloud_ptr, cloud_normals1, cloud_normals2);
  else if (interaction_customization)
    viewer = interactionCustomizationVis();

  // 6. 主循环：保持窗口刷新
  while (!viewer->wasStopped())
  {
    viewer->spinOnce(100);                                      // 处理事件，100ms超时
    boost::this_thread::sleep(boost::posix_time::microseconds(100000)); // 100ms休眠
  }

  return 0;
}