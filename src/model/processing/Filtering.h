#ifndef SWAG_SCANNER_FILTERING_H
#define SWAG_SCANNER_FILTERING_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * Utility file that contains functions for filtering point clouds.
 */
namespace filtering {

    /**
     * Crop a cloud based on parameters.
     * Note that x,y,z values is based off of camera reference frame.
     * @param cloud cloud you want to crop.
     * @return the cropped cloud.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                   float minX, float maxX,
                                                   float minY, float maxY,
                                                   float minZ, float maxZ);

    /**
     * Downsample the given cloud using a voxel grid filter and leaf size.
     * @param cloud the cloud you want to filter.
     * @param leafSize size of leaf.
     * @return the filtered cloud.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_grid_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                          float leafSize);
}

#endif //SWAG_SCANNER_FILTERING_H
