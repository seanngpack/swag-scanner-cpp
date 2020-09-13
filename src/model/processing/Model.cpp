#include "Model.h"
#include "Point.h"
#include "Normal.h"
#include "Plane.h"
#include "Depth.h"
#include "Filtering.h"
#include "Segmentation.h"
#include "Calibration.h"
#include "Visualizer.h"
#include "Algorithms.h"
#include "Registration.h"
#include "CameraTypes.h"
#include <pcl/common/transforms.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

model::Model::Model() {}


std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
model::Model::create_point_cloud(const std::vector<uint16_t> &depth_frame,
                                 const camera::intrinsics &intrinsics) {

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> point_cloud = depth::create_point_cloud(depth_frame, intrinsics);
    return point_cloud;
}

std::shared_ptr<pcl::PointCloud<pcl::Normal>> model::Model::estimate_normal_cloud(
        const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud) {

    auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);

    ne.setInputCloud(cloud);
    std::cout << "estimating normal cloud" << std::endl;
    std::cout << cloud->size() << std::endl;
    ne.compute(*normals);
    return normals;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr model::Model::compute_local_features(
        const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
        const std::shared_ptr<pcl::PointCloud<pcl::Normal>> &normal_cloud) {
    auto features = std::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();
    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud(cloud);
    fpfh_est.setInputNormals(normal_cloud);
    fpfh_est.setSearchMethod(tree);
    fpfh_est.setRadiusSearch(.05);
    fpfh_est.compute(*features);
    return features;
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
model::Model::crop_cloud(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                         float minX, float maxX,
                         float minY, float maxY,
                         float minZ, float maxZ) {
    return filtering::crop_cloud(cloud, minX, maxX, minY, maxY, minZ, maxZ);
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
model::Model::voxel_grid_filter(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                float leafSize) {
    return filtering::voxel_grid_filter(cloud, leafSize);
}

std::vector<equations::Plane>
model::Model::get_calibration_planes_coefs(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                           bool visual_flag) {
    return segmentation::get_calibration_planes_coefs(cloud, visual_flag);
}

std::vector<float> model::Model::get_plane_coefs(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud) {
    return segmentation::get_plane_coefs(cloud);
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
model::Model::remove_plane(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloudIn) {
    return segmentation::remove_plane(cloudIn);
}

equations::Normal model::Model::calculate_axis_dir(const std::vector<equations::Plane> &ground_planes) {
    equations::Normal g_n;
    for (auto &g: ground_planes) {
        g_n.A += g.A;
        g_n.B += g.B;
        g_n.C += g.C;
    }
    g_n.A /= ground_planes.size();
    g_n.B /= ground_planes.size();
    g_n.C /= ground_planes.size();
    return g_n;
}

equations::Plane model::Model::average_planes(const std::vector<equations::Plane> &planes) {
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

equations::Point model::Model::calculate_center_pt(const equations::Normal &axis_dir,
                                                   const std::vector<equations::Plane> &upright_planes) {

    Eigen::MatrixXd A = calibration::build_A_matrix(axis_dir, upright_planes);
    Eigen::MatrixXd b = calibration::build_b_matrix(axis_dir, upright_planes);

    equations::Point center = calibration::calculate_center_pt(A, b);

    return center;
}

pcl::PointXYZ model::Model::refine_center_pt(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                             const pcl::PointXYZ &pt,
                                             const equations::Plane &plane,
                                             double delta) {
    pcl::PointXYZ plane_pt = algos::find_point_in_plane(cloud, plane, delta);
    return algos::project_point_to_plane(pt, plane_pt, plane.get_normal());
}

pcl::PointCloud<pcl::PointXYZ>
model::Model::rotate_cloud_about_line(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                      const std::vector<float> &line_point,
                                      const std::vector<float> &line_direction,
                                      float theta) {
    pcl::PointCloud<pcl::PointXYZ> transformed;

    transformed.resize(cloud->size());

    for (int i = 0; i < cloud->size(); i++) {
        transformed.points[i] = algos::rotate_point_about_line(cloud->points[i],
                                                               line_point,
                                                               line_direction,
                                                               theta);
    }
    return transformed;
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
model::Model::transform_cloud_to_world(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                       const pcl::PointXYZ &center,
                                       const equations::Normal &ground_normal) {
    auto result = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    Eigen::Matrix4f transform = algos::calc_transform_to_world_matrix(center, ground_normal);
    pcl::transformPointCloud(*cloud, *result, transform);
    return result;
}

Eigen::Matrix4f model::Model::icp_register_pair_clouds(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud_in,
                                                       const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud_out,
                                                       std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &transformed_cloud) {
    return registration::icp_register_pair_clouds(cloud_in, cloud_out, transformed_cloud);

}


void model::Model::sac_align_pair_clouds(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloudIn,
                                         const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloudTarget,
                                         const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloudAligned,
                                         Eigen::Matrix4f &transformation) {

    pcl::PointCloud<pcl::Normal>::Ptr cloudInNormal = estimate_normal_cloud(cloudIn);
    pcl::PointCloud<pcl::Normal>::Ptr cloudTargetNormal = estimate_normal_cloud(cloudTarget);
    auto cloudInFeatures = std::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();
    auto cloudTargetFeatures = std::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();
    registration::sac_align_pair_clouds(cloudIn, cloudTarget, cloudInFeatures, cloudTargetFeatures,
                                        cloudAligned, transformation);
}


