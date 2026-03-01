#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>

int main(int argc, char** argv)
{
    // ==============================
    // 1. 创建或加载点云数据
    // ==============================
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    // 示例：生成一个简单的平面点云
    for (float x = -1.0f; x <= 1.0f; x += 0.02f)
    {
        for (float y = -1.0f; y <= 1.0f; y += 0.02f)
        {
            pcl::PointXYZ point;
            point.x = x;
            point.y = y;
            point.z = 0.0f; // 平面 z=0
            cloud->points.push_back(point);
        }
    }
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;

    std::cout << "Generated " << cloud->size() << " points." << std::endl;

    // ==============================
    // 2. 估计法向量
    // ==============================
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    // 使用 KD-Tree 加速邻域搜索
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);

    // 使用半径为 3cm 的邻域来估计法向量
    ne.setRadiusSearch(0.03);
    ne.compute(*normals);

    std::cout << "Computed " << normals->size() << " normals." << std::endl;

    // ==============================
    // 3. 创建 FPFH 估计器并设置参数
    // ==============================
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(normals);

    // 设置搜索方法（KD-Tree）
    fpfh.setSearchMethod(tree);  // 可复用前面创建的 tree

    // 输出数据集
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>);

    // 设置搜索半径：必须大于法向量估计的半径（这里是 0.03）
    fpfh.setRadiusSearch(0.05);  // 5cm

    // ==============================
    // 4. 计算 FPFH 描述子
    // ==============================
    fpfh.compute(*fpfhs);

    // ==============================
    // 5. 验证输出
    // ==============================
    std::cout << "FPFH descriptors computed: " << fpfhs->size() << std::endl;
    std::cout << "Each descriptor has " << fpfhs->points[0].histogram[32] + 1 << " dimensions (should be 33)." << std::endl;

    // 可选：保存到文件
    pcl::io::savePCDFileASCII("fpfh_descriptors.pcd", *fpfhs);
    std::cout << "Saved FPFH descriptors to fpfh_descriptors.pcd" << std::endl;

    return 0;
}
