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
     * Get the plane coefficients of the base of the given cloud.
     * @param cloud input cloud.
     * @return coefficients vector of size 4 of the cloud.
     */
    std::vector<float> get_plane_coefs(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud);
}

#endif //SWAG_SCANNER_SEGMENTATION_H
