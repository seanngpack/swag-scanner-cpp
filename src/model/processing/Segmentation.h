#ifndef SWAG_SCANNER_SEGMENTATION_H
#define SWAG_SCANNER_SEGMENTATION_H

#include "Plane.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


/**
 * Contains functions for segmentation (right now just planes)
 */
namespace segmentation {
    /**
     * Given a cloud with plane, detect the plane using RANSAC, remove it, and return
     * a cloud without the plane.
     * @param cloud cloud with plane.
     * @return cloud without plane.
     */
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
    remove_plane(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud);


    /**
     * Given a calibration cloud, extract the upright and ground plane.
     *
     * @param cloud calibration cloud.
     * @param visual_flag flag to show visual segmentation.
     * @return a vector of coefficients for the two planes.
     * First element in vector is ground plane. Second element is upright plane.
     */
    std::vector<equations::Plane>
    get_calibration_planes_coefs(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                 bool visual_flag = false);


    /**
     * Get the plane coefficients of the base of the given cloud.
     * @param cloud input cloud.
     * @return coefficients vector of size 4 of the cloud.
     */
    std::vector<float> get_plane_coefs(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud);
}

#endif //SWAG_SCANNER_SEGMENTATION_H
