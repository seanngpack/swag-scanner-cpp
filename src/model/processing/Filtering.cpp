#include "Filtering.h"
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
filtering::crop_cloud(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                      float minX, float maxX,
                      float minY, float maxY,
                      float minZ, float maxZ) {
    auto cropped_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setKeepOrganized(1);
    boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter.setInputCloud(cloud);
    boxFilter.filter(*cropped_cloud);
    return cropped_cloud;
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
filtering::voxel_grid_filter(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                             float leafSize) {
    auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
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

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
filtering::remove_outliers(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                           float mean_k,
                           float thesh_mult) {
    auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(thesh_mult);
    sor.filter(*filtered);

    return filtered;
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
filtering::remove_nan(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud) {
    auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *filtered, indices);
    return filtered;
}
