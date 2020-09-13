#ifndef SWAG_SCANNER_ALGORITHMS_H
#define SWAG_SCANNER_ALGORITHMS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

namespace camera {
    class intrinsics;
}

namespace equations {
    class Normal;

    class Plane;

    class Point;
}

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
                                           const camera::intrinsics &intrinsics);

    /**
     * Project point to plane
     *
     * @param pt point you want to project.
     * @param plane_pt a point on the plane.
     * @param plane the plane you want to project onto.
     * @param normal normal of the plane.
     * @return projected point on the plane.
     */
    pcl::PointXYZ project_point_to_plane(const pcl::PointXYZ &pt,
                                         const pcl::PointXYZ &plane_pt,
                                         const equations::Normal &normal);

    /**
     * Find a point lying on the given plane in the cloud.
     *
     * @param cloud cloud.
     * @param plane plane.
     * @param delta error threshold for finding the point.
     * @return point in the plane or point of 0,0,0.
     */
    pcl::PointXYZ find_point_in_plane(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                      const equations::Plane &plane,
                                      double delta = .00001);

    /**
     * Check if a point is inside a plane with given threshold.
     *
     * @param pt point you want to check.
     * @param plane plane the pt belongs to.
     * @param delta error threshold.
     * @return true if the point is in the plane, false otherwise.
     */
    bool check_point_in_plane(const pcl::PointXYZ &pt,
                              const equations::Plane &plane,
                              double delta);

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
    pcl::PointXYZ rotate_point_about_line(const pcl::PointXYZ &point,
                                          const std::vector<float> &line_point,
                                          const std::vector<float> &line_direction,
                                          float theta);

    /**
     * Calculate the transformation matrix from center of turntable to origin (0,0,0).
     * @param center center of turntable.
     * @param ground_normal normal vector of ground plane.
     * @return
     */
    Eigen::Matrix4f calc_transform_to_world_matrix(const pcl::PointXYZ &center,
                                                   const equations::Normal &ground_normal);
}

#endif //SWAG_SCANNER_ALGORITHMS_H
