#include <pcl/range_image/range_image.h>   // 引入 PCL 深度图像相关头文件

int main (int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ> pointCloud;   // 创建 XYZ 点云对象

  // 生成数据：在 y-z 平面上每隔 1 cm 采样，x 固定为 2 - y
  for (float y = -0.5f; y <= 0.5f; y += 0.01f) {
    for (float z = -0.5f; z <= 0.5f; z += 0.01f) {
      pcl::PointXYZ point;           // 创建一个点
      point.x = 2.0f - y;            // x = 2 - y
      point.y = y;                   // y 保持不变
      point.z = z;                   // z 保持不变
      pointCloud.points.push_back(point); // 加入点云
    }
  }

  // 设置点云宽度和高度（无组织点云 height = 1）
  pointCloud.width  = (uint32_t) pointCloud.points.size();
  pointCloud.height = 1;

  // ----------- 以下参数用于生成深度图 -----------
  float angularResolution = (float) (1.0f * (M_PI/180.0f));   // 角分辨率：1° → 弧度
  float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f)); // 水平视野：360° → 弧度
  float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f)); // 垂直视野：180° → 弧度

  // 传感器位姿：原点，无旋转
  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);

  // 坐标系：以相机坐标系为准
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;

  float noiseLevel = 0.00;   // 无噪声
  float minRange   = 0.0f;   // 最小测距 0
  int   borderSize = 1;      // 边界像素 1

  // 创建并生成深度图像
  pcl::RangeImage rangeImage;
  rangeImage.createFromPointCloud(pointCloud,
                                  angularResolution, maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame,
                                  noiseLevel, minRange, borderSize);

  // 终端输出：深度图像信息
  std::cout << "生成的深度图像：\n" << rangeImage << "\n";
}