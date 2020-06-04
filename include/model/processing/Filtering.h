/**
 * Utility file that contains functions for filtering point clouds.
 */
#ifndef SWAG_SCANNER_FILTERING_H
#define SWAG_SCANNER_FILTERING_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

namespace filtering {

    inline void crop_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                           float minX, float maxX,
                           float minY, float maxY,
                           float minZ, float maxZ) {
        pcl::CropBox<pcl::PointXYZ> boxFilter;
        boxFilter.setKeepOrganized(1);
        boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
        boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
        boxFilter.setInputCloud(cloud);
        boxFilter.filter(*cloud);
    }


    /**
     *
     * @param cloud
     * @param leadSize
     * @return
     */
    inline pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_grid_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                                 float leafSize) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> grid;
        std::cout << "PointCloud before filtering: " << cloud->width * cloud->height
                  << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;
        grid.setInputCloud(cloud);
        grid.setLeafSize(leafSize, leafSize, leafSize);
        grid.filter(*cloud_filtered);
        std::cout << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
                  << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;
        return cloud_filtered;
    }
}

#endif //SWAG_SCANNER_FILTERING_H
