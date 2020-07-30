#ifndef SWAG_SCANNER_ALGORITHMS_H
#define SWAG_SCANNER_ALGORITHMS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include "CameraTypes.h"
#include "Normal.h"
#include "Plane.h"
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

/**
 * Assortment of hand-made algos for drop&play.
 * List may not be big because I may implement them directly by the
 * function definitions.
 */
namespace algos {

    /**
     * Given three points, deproject their pixel coordinates to space coordinates and
     * then save to a PointXYZ format.
     * @param x pixel x.
     * @param y pixel y.
     * @param z depth (unconverted).
     * @return a PointXYZ object with the deprojected point in real space.
     */
    pcl::PointXYZ deproject_pixel_to_point(float x_pixel,
                                           float y_pixel,
                                           float z,
                                           const camera::ss_intrinsics *intrinsics);

    /**
     * Given a copy of a point from the pointcloud, a point that a line passes through,
     * and a direction vector, rotate the pointcloud point about that line and return
     * a copy of the new point.
     * Equation derived by Glenn Murray.
     *
     * @param point the point you want to rotate.
     * @param line_point point on the line.
     * @param line_direction direction vector (normalized) of the axis.
     * @return a new rotated point.
     */
    pcl::PointXYZ rotate_point_about_line(pcl::PointXYZ point,
                                          std::vector<float> line_point,
                                          std::vector<float> line_direction,
                                          float theta);

    /**
     * Calculate the transformation matrix from center of turntable to origin (0,0,0).
     * @param center center of turntable.
     * @param ground_normal normal vector of ground plane.
     * @return
     */
    Eigen::Matrix4f calc_transform_to_world_matrix(pcl::PointXYZ center,
                                                   equations::Normal ground_normal);
}

#endif //SWAG_SCANNER_ALGORITHMS_H
