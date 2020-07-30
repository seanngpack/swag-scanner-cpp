/**
 * Assortment of hand-made algos for drop&play.
 * List may not be big because I may implement them directly by the
 * function definitions.
 */
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
    inline pcl::PointXYZ rotate_point_about_line(pcl::PointXYZ point,
                                                 std::vector<float> line_point,
                                                 std::vector<float> line_direction,
                                                 float theta) {
        float x = point.x;
        float y = point.y;
        float z = point.z;
        float a = line_point[0];
        float b = line_point[1];
        float c = line_point[2];
        float u = line_direction[0];
        float v = line_direction[1];
        float w = line_direction[2];

        pcl::PointXYZ p;
        p.x = (a * (v * v + w * w) - u * (b * v + c * w - u * x - v * y - w * z)) * (1 - cos(theta)) + x * cos(theta) +
              (-c * v + b * w - w * y + v * z) * sin(theta);
        p.y = (b * (u * u + w * w) - v * (a * u + c * w - u * x - v * y - w * z)) * (1 - cos(theta)) + y * cos(theta) +
              (c * u - a * w + w * x - u * z) * sin(theta);
        p.z = (c * (u * u + v * v) - w * (a * u + b * v - u * x - v * y - w * z)) * (1 - cos(theta)) + z * cos(theta) +
              (-b * u + a * v - v * x + u * y) * sin(theta);

        return p;
    }

    inline Eigen::Matrix4f calc_transform_to_world_matrix(pcl::PointXYZ center,
                                                       equations::Normal ground_normal) {
        Eigen::Vector3f translation_vect(center.getVector3fMap());
        translation_vect = -translation_vect;
        Eigen::Translation<float, 3> translation(translation_vect);

        float a_dot_b = Eigen::Vector3f(ground_normal.A,
                                        ground_normal.B,
                                        ground_normal.C).dot(
                Eigen::Vector3f(0, 0, 1));
        float angle = -acos(a_dot_b);
        Eigen::AngleAxis<float> rotation(angle, Eigen::Vector3f(1, 0, 0));
        Eigen::Transform<float, 3, Eigen::Affine> transform = rotation * translation;
        return transform.matrix();
    }
}

#endif //SWAG_SCANNER_ALGORITHMS_H
