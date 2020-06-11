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

    /**
     * Crop a cloud based on parameters. Note that x,y,z values is based off of camera
     * reference frame.
     * @param cloud cloud you want to crop.
     * @return the cropped cloud.
     */
    inline pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
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


    /**
     * Downsample the given cloud using a voxel grid filter and leaf size.
     * @param cloud the cloud you want to filter.
     * @param leafSize size of leaf.
     * @return the filtered cloud.
     */
    inline pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_grid_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
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
}

#endif //SWAG_SCANNER_FILTERING_H
