/**
 * Assortment of hand-made algos for drop&play.
 * List may not be big because I may implement them directly by the
 * function definitions.
 */
#ifndef SWAG_SCANNER_ALGORITHMS_H
#define SWAG_SCANNER_ALGORITHMS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "CameraTypes.h"
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

namespace algos {

    /**
     * Given three points, deproject their pixel coordinates to space coordinates and
     * then save to a PointXYZ format.
     * @param x pixel x.
     * @param y pixel y.
     * @param z depth (unconverted).
     * @return a PointXYZ object with the deprojected point in real space.
     */
    inline pcl::PointXYZ deproject_pixel_to_point(float x_pixel,
                                           float y_pixel,
                                           float z,
                                           const camera::ss_intrinsics *intrinsics) {
        float depth = z * intrinsics->depth_scale;
        float x = (x_pixel - intrinsics->ppx) / intrinsics->fx;
        float y = (y_pixel - intrinsics->ppy) / intrinsics->fy;
        float ux = x * depth;
        float uy = y * depth;

        pcl::PointXYZ point = pcl::PointXYZ(ux, uy, depth);
        return point;
    }


#endif //SWAG_SCANNER_ALGORITHMS_H
