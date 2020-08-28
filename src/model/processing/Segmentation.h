#ifndef SWAG_SCANNER_SEGMENTATION_H
#define SWAG_SCANNER_SEGMENTATION_H

#include "../equations/Plane.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


/**
 * Contains functions for segmentation (right now just planes)
 */
namespace segmentation {
    /**
     * Given a cloud with plane, detect the plane using RANSAC, remove it, and return
     * a cloud without the plane.
     * @param cloudIn cloud with plane.
     * @return cloud without plane.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr remove_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudIn);


    /**
     * Given a calibration cloud, extract the ground and upright plane.
     *
     * @param cloud calibration cloud.
     * @return a vector of coefficients for the two planes.
     * First element in vector is upright plane. Second element is ground plane.
     */
    std::vector<equations::Plane> get_calibration_planes_coefs(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /**
     * Get the plane coefficients of the base of the given cloud.
     * @param cloud input cloud.
     * @return coefficients vector of size 4 of the cloud.
     */
    std::vector<float> get_plane_coefs(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
}

#endif //SWAG_SCANNER_SEGMENTATION_H
