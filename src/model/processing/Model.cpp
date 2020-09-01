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


std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> model::Model::create_point_cloud(const std::vector<uint16_t> &depth_frame,
                                                                     const camera::intrinsics intrinsics) {

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> point_cloud = depth::create_point_cloud(depth_frame, intrinsics);
    return point_cloud;
}

pcl::PointCloud<pcl::Normal>::Ptr model::Model::estimate_normal_cloud(
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud) {

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);

    ne.setInputCloud(cloud);
    std::cout << "estimating normal cloud" << std::endl;
    std::cout << cloud->size() << std::endl;
    ne.compute(*normals);
    return normals;
}

void model::Model::compute_local_features(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud,
                                          pcl::PointCloud<pcl::Normal>::Ptr normalCloud,
                                          pcl::PointCloud<pcl::FPFHSignature33>::Ptr features) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr searchMethod(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud(cloud);
    fpfh_est.setInputNormals(normalCloud);
    fpfh_est.setSearchMethod(searchMethod);
    fpfh_est.setRadiusSearch(.05);
    fpfh_est.compute(*features);
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> model::Model::crop_cloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud,
                                                             float minX, float maxX,
                                                             float minY, float maxY,
                                                             float minZ, float maxZ) {
    return filtering::crop_cloud(cloud, minX, maxX, minY, maxY, minZ, maxZ);
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> model::Model::voxel_grid_filter(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud,
                                                                    float leafSize) {
    return filtering::voxel_grid_filter(cloud, leafSize);
}

std::vector<equations::Plane> model::Model::get_calibration_planes_coefs(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud) {
    return segmentation::get_calibration_planes_coefs(cloud);
}

std::vector<float> model::Model::get_plane_coefs(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud) {
    return segmentation::get_plane_coefs(cloud);
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> model::Model::remove_plane(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloudIn) {
    return segmentation::remove_plane(cloudIn);
}

equations::Normal model::Model::calculate_axis_dir(std::vector<equations::Plane> ground_planes) {
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

equations::Point model::Model::calculate_center_pt(equations::Normal axis_dir,
                                                   std::vector<equations::Plane> upright_planes) {

    Eigen::MatrixXd A = calibration::build_A_matrix(axis_dir, upright_planes);
    Eigen::MatrixXd b = calibration::build_b_matrix(axis_dir, upright_planes);

    equations::Point center = calibration::calculate_center_pt(A, b);

    return center;
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> model::Model::rotate_cloud_about_line(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud,
                                                                          std::vector<float> line_point,
                                                                          std::vector<float> line_direction,
                                                                          float theta) {
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> transformed(new pcl::PointCloud<pcl::PointXYZ>);

    transformed->resize(cloud->size());

    for (int i = 0; i < cloud->size(); i++) {
        transformed->points[i] = algos::rotate_point_about_line(cloud->points[i],
                                                                line_point,
                                                                line_direction,
                                                                theta);
    }
    return transformed;
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> model::Model::transform_cloud_to_world(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud,
                                                                           pcl::PointXYZ center,
                                                                           equations::Normal ground_normal) {
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> result(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f transform = algos::calc_transform_to_world_matrix(center, ground_normal);
    pcl::transformPointCloud(*cloud, *result, transform);
    return result;
}

Eigen::Matrix4f model::Model::icp_register_pair_clouds(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudIn,
                                                       std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudOut,
                                                       std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> transformedCloud) {
    return registration::icp_register_pair_clouds(cloudIn, cloudOut, transformedCloud);

}


void model::Model::sac_align_pair_clouds(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudIn,
                                         std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudTarget,
                                         std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudAligned,
                                         Eigen::Matrix4f &transformation) {

    pcl::PointCloud<pcl::Normal>::Ptr cloudInNormal = estimate_normal_cloud(cloudIn);
    pcl::PointCloud<pcl::Normal>::Ptr cloudTargetNormal = estimate_normal_cloud(cloudTarget);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloudInFeatures(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloudTargetFeatures(new pcl::PointCloud<pcl::FPFHSignature33>);

    registration::sac_align_pair_clouds(cloudIn, cloudTarget, cloudInFeatures, cloudTargetFeatures,
                                        cloudAligned, transformation);
}

model::Model::~Model() {
    std::cout << "calling model destructor \n";;
}

