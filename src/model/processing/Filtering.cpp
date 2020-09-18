#include "Filtering.h"
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

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
