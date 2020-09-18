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
model::Model::voxel_grid_filter(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                float leafSize) {
    return filtering::voxel_grid_filter(cloud, leafSize);
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
model::Model::remove_outliers(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                              float mean_k,
                              float thresh_mult) {
    return filtering::remove_outliers(cloud, mean_k, thresh_mult);
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
model::Model::remove_nan(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud) {
    return filtering::remove_nan(cloud);
}



std::vector<float> model::Model::get_plane_coefs(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud) {
    return segmentation::get_plane_coefs(cloud);
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
model::Model::remove_plane(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloudIn) {
    return segmentation::remove_plane(cloudIn);
}









std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
model::Model::transform_cloud_to_world(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                       const pcl::PointXYZ &center,
                                       const equations::Normal &rotation_axis) {
    auto result = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    Eigen::Matrix4f transform = algos::calc_transform_to_world_matrix(center, rotation_axis);
    pcl::transformPointCloud(*cloud, *result, transform);
    return result;
}

Eigen::Matrix4f model::Model::icp_register_pair_clouds(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud_in,
                                                       const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud_target,
                                                       std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &transformed_cloud) {
    return registration::icp_register_pair_clouds(cloud_in, cloud_target, transformed_cloud);

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



