#include "Algorithms.h"
#include "Normal.h"
#include "Plane.h"
#include "CameraTypes.h"

pcl::PointXYZ algos::deproject_pixel_to_point(float x_pixel,
                                              float y_pixel,
                                              float z,
                                              const camera::intrinsics &intrinsics) {
    float depth = z * intrinsics.depth_scale;
    float x = (x_pixel - intrinsics.ppx) / intrinsics.fx;
    float y = (y_pixel - intrinsics.ppy) / intrinsics.fy;
    float ux = x * depth;
    float uy = y * depth;

    pcl::PointXYZ point = pcl::PointXYZ(ux, uy, depth);
    return point;
}

pcl::PointXYZ algos::rotate_point_about_line(const pcl::PointXYZ &point,
                                             const std::vector<float> &line_point,
                                             const std::vector<float> &line_direction,
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

Eigen::Matrix4f algos::calc_transform_to_world_matrix(const pcl::PointXYZ &center,
                                                      const equations::Normal &ground_normal) {
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