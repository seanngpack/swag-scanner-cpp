#include "Filtering.h"
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr filtering::crop_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                          float minX, float maxX,
                                                          float minY, float maxY,
                                                          float minZ, float maxZ) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr croppedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setKeepOrganized(1);
    boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter.setInputCloud(cloud);
    boxFilter.filter(*croppedCloud);
    return croppedCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr filtering::voxel_grid_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                                 float leafSize) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    std::cout << "PointCloud before filtering: " << cloud->width * cloud->height
              << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;
    grid.setInputCloud(cloud);
    grid.setLeafSize(leafSize, leafSize, leafSize);
    grid.filter(*cloud_filtered);
    std::cout << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
              << " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;
    return cloud_filtered;
}