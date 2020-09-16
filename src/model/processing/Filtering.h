#ifndef SWAG_SCANNER_FILTERING_H
#define SWAG_SCANNER_FILTERING_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

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
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
    crop_cloud(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
               float minX, float maxX,
               float minY, float maxY,
               float minZ, float maxZ);

    /**
     * Downsample the given cloud using a voxel grid filter and leaf size.
     * @param cloud the cloud you want to filter.
     * @param leafSize size of leaf.
     * @return the filtered cloud.
     */
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
    voxel_grid_filter(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                      float leafSize);

    /**
     * Remove outliers from cloud.
     *
     * @param cloud cloud to filter.
     * @param mean_k number of neighbors to analyze.
     * @param thesh multipler for standard deviation, members outside st will be removed.
     * @return filtered cloud.
     */
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
    remove_outliers(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                    float mean_k,
                    float thesh_mult);

    /**
     * Remove NaN points from cloud.
     *
     * @param cloud cloud to remove points from.
     * @return cloud without points.
     */
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
    remove_nan(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud);
}

#endif //SWAG_SCANNER_FILTERING_H
