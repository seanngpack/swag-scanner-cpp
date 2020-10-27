#include "Algorithms.h"
#include "Normal.h"
#include "Plane.h"
#include "CameraTypes.h"
#include "Logger.h"
#include <pcl/common/transforms.h>

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

pcl::PointXYZ algos::project_point_to_plane(const pcl::PointXYZ &pt,
                                            const pcl::PointXYZ &plane_pt,
                                            const equations::Normal &normal) {
    double a = normal.A;
    double b = normal.B;
    double c = normal.C;
    double d = plane_pt.x;
    double e = plane_pt.y;
    double f = plane_pt.z;

    double x = pt.x;
    double y = pt.y;
    double z = pt.z;

    double t = (a * d - a * x + b * e - b * y + c * f - c * z) / (a * a + b * b + c * c);

    pcl::PointXYZ point = pcl::PointXYZ(x + t * a,
                                        y + t * b,
                                        z + t * c);
    return point;
}

bool algos::check_point_in_plane(const pcl::PointXYZ &pt,
                                 const equations::Plane &plane,
                                 double delta) {


    double lhs = pt.x * plane.A + pt.y * plane.B + pt.z * plane.C + plane.D;
    if (lhs > -delta && lhs < delta) {
        logger::info("found point for projection with error: " + std::to_string(lhs));
        return true;
    }
    return false;
}

pcl::PointXYZ algos::find_point_in_plane(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                         const equations::Plane &plane,
                                         double delta) {
    for (const auto &pt: cloud->points) {
        if (check_point_in_plane(pt, plane, delta)) {
            return pt;
        }
    }
    logger::error("cannot find point in threshold, try loosening it");
    return pcl::PointXYZ(0, 0, 0);
}


pcl::PointCloud<pcl::PointXYZ>
algos::rotate_cloud_about_line(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                               const std::vector<float> &pt,
                               const std::vector<float> &line_direction,
                               float theta) {
    pcl::PointCloud<pcl::PointXYZ> transformed;

    transformed.resize(cloud->size());

    for (int i = 0; i < cloud->size(); i++) {
        transformed.points[i] = rotate_point_about_line(cloud->points[i],
                                                        pt,
                                                        line_direction,
                                                        theta);
    }
    return transformed;
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

pcl::PointCloud<pcl::PointXYZ>
algos::rotate_cloud_about_z_axis(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud, float theta) {
    pcl::PointCloud<pcl::PointXYZ> rotated;
    Eigen::Affine3f transform(Eigen::Affine3f::Identity());
    // note, rotating in negative direction
    transform.rotate(Eigen::AngleAxisf(-(theta * M_PI) / 180, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*cloud, rotated, transform);
    return rotated;
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

void algos::transform_cloud_to_world(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                     const pcl::PointXYZ &center,
                                     const equations::Normal &ground_normal) {
    Eigen::Matrix4f transform = calc_transform_to_world_matrix(center, ground_normal);
    pcl::transformPointCloud(*cloud, *cloud, transform);
}

equations::Plane algos::average_planes(const std::vector<equations::Plane> &planes) {
    equations::Plane avg;
    for (auto &g: planes) {
        avg.A += g.A;
        avg.B += g.B;
        avg.C += g.C;
        avg.D += g.D;
    }
    avg.A /= planes.size();
    avg.B /= planes.size();
    avg.C /= planes.size();
    avg.D /= planes.size();
    return avg;
}