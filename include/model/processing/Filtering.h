/**
 * Utility file that contains functions for filtering point clouds.
 */
#ifndef SWAG_SCANNER_FILTERING_H
#define SWAG_SCANNER_FILTERING_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

namespace filtering {

    inline void crop_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
            float minX, float maxX,
            float minY, float maxY,
            float minZ, float maxZ) {
        pcl::CropBox<pcl::PointXYZ> boxFilter;
        boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
        boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
        boxFilter.setInputCloud(cloud);
        boxFilter.filter(*cloud);
    }
}

#endif //SWAG_SCANNER_FILTERING_H
