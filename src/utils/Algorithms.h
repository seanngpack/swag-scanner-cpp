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
     * Find a point lying on the given plane in the calibration.
     *
     * @param cloud calibration.
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
     * Rotate a point calibration about a line.
     *
     * @param cloud the calibration you want to rotate. Must be an unorganized calibration.
     * @param pt a point on the axis you want to rotate about.
     * @param line_direction direction vector for the line (normalized)
     * @param theta angle in radians you want to rotate.
     * @return the rotated calibration.
     */
    pcl::PointCloud<pcl::PointXYZ>
    rotate_cloud_about_line(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                            const std::vector<float> &pt,
                            const std::vector<float> &line_direction,
                            float theta);

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
      * Rotate calibration about the z-axis.
      *
      * @param cloud calibration to rotate.
      * @param theta rotation degree.
      * @return
      */
    pcl::PointCloud<pcl::PointXYZ>
    rotate_cloud_about_z_axis(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                              float theta);

    /**
     * Calculate the transformation matrix from center of turntable to origin (0,0,0).
     *
     * @param center center of turntable.
     * @param ground_normal normal vector of ground plane.
     * @return matrix of the transformation.
     */
    Eigen::Matrix4f calc_transform_to_world_matrix(const pcl::PointXYZ &center,
                                                   const equations::Normal &ground_normal);

    /**
     * Transform cloud to world coordinate and return a copy.
     */
    void transform_cloud_to_world(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                  const pcl::PointXYZ &center,
                                  const equations::Normal &ground_normal);


    /**
     * Given a vector of planes, average them.
     *
     * @param planes planes.
     * @return average of the planes.
     */
    equations::Plane average_planes(const std::vector<equations::Plane> &planes);
}

#endif //SWAG_SCANNER_ALGORITHMS_H
