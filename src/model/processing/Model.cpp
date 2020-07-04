#include "Model.h"
#include "Depth.h"
#include "Filtering.h"
#include "Segmentation.h"
#include "Calibration.h"

model::Model::Model() {}


pcl::PointCloud<pcl::PointXYZ>::Ptr model::Model::create_point_cloud(const uint16_t *depth_frame,
                                                                     const camera::ss_intrinsics *intrinsics) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = depth::create_point_cloud(depth_frame, intrinsics);
    return point_cloud;
}

pcl::PointCloud<pcl::Normal>::Ptr model::Model::estimate_normal_cloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

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

void model::Model::compute_local_features(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
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

pcl::PointCloud<pcl::PointXYZ>::Ptr model::Model::crop_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                             float minX, float maxX,
                                                             float minY, float maxY,
                                                             float minZ, float maxZ) {
    return filtering::crop_cloud(cloud, minX, maxX, minY, maxY, minZ, maxZ);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr model::Model::voxel_grid_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                                    float leafSize) {
    return filtering::voxel_grid_filter(cloud, leafSize);
}

std::vector<float> model::Model::get_plane_coefs(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    return segmentation::get_plane_coefs(cloud);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr model::Model::remove_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudIn) {
    return segmentation::remove_plane(cloudIn);
}

equations::Point model::Model::calculate_center_pt(std::vector<equations::Plane> ground_planes,
                                                   std::vector<equations::Plane> upright_planes) {
    equations::Normal g_n;
    for (auto &g: ground_planes) {
        g_n.A += g.A;
        g_n.B += g.B;
        g_n.C += g.C;
    }
    g_n.A /= ground_planes.size();
    g_n.B /= ground_planes.size();
    g_n.C /= ground_planes.size();

    Eigen::MatrixXd A = calibration::build_A_matrix(g_n, upright_planes);
    Eigen::MatrixXd b = calibration::build_b_matrix(g_n, upright_planes);

    equations::Point center = calibration::calculate_center_pt(A, b);

    return center;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr model::Model::rotate_cloud_about_line(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                                          std::vector<float> line_point,
                                                                          std::vector<float> line_direction,
                                                                          float theta) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);

    transformed->resize(cloud->size());

    for (int i = 0; i < cloud->size(); i++) {
        transformed->points[i] = algos::rotate_point_about_line(cloud->points[i],
                                                                line_point,
                                                                line_direction,
                                                                theta);
    }
    return transformed;
}

Eigen::Matrix4f model::Model::icp_register_pair_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn,
                                                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut,
                                                       pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud) {
    return registration::icp_register_pair_clouds(cloudIn, cloudOut, transformedCloud);

}


void model::Model::sac_align_pair_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTarget,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAligned,
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









