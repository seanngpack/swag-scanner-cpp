#include "CalibrationModel.h"
#include "Normal.h"
#include "Plane.h"
#include "Equations.h"
#include "Algorithms.h"
#include "Visualizer.h"
#include "CloudType.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>

model::CalibrationModel::CalibrationModel() :
        file_handler() {}

void model::CalibrationModel::set_calibration(const std::string &cal_name) {
    file_handler.set_calibration(cal_name);
    clouds = file_handler.load_clouds(CloudType::Type::CALIBRATION);
    ground_planes.clear();
    upright_planes.clear();
}

void model::CalibrationModel::save_cloud(const std::string &cloud_name) {
    auto cloud = clouds[clouds_map[cloud_name]];
    file_handler.save_cloud(cloud, cloud_name, CloudType::Type::CALIBRATION);
}


pcl::PointXYZ model::CalibrationModel::calculate_center_point() {
    // use clouds to find ground and upright planes.
    for (const auto &c : clouds) {
        std::vector<equations::Plane> coeffs = get_calibration_planes_coefs(c);
        ground_planes.emplace_back(coeffs[0]);
        upright_planes.emplace_back(coeffs[1]);
    }

    // calculate rotation axis direction and use the calculated data so far to construct matrices
    axis_of_rotation = calculate_axis_dir(ground_planes);
    Eigen::MatrixXd A = build_A_matrix(axis_of_rotation, upright_planes);
    Eigen::MatrixXd b = build_b_matrix(axis_of_rotation, upright_planes);

    // solve!!
    Eigen::MatrixXd sol_mat = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    std::vector<double> sol_vec(sol_mat.data(), sol_mat.data() + sol_mat.rows() * sol_mat.cols());
    return pcl::PointXYZ(sol_vec[0], sol_vec[1], sol_vec[2]);
}

pcl::PointXYZ model::CalibrationModel::calculate_center_point(const equations::Normal &axis_dir,
                                                              const std::vector<equations::Plane> &upright_planes) {
    Eigen::MatrixXd A = build_A_matrix(axis_dir, upright_planes);
    Eigen::MatrixXd b = build_b_matrix(axis_dir, upright_planes);

    Eigen::MatrixXd sol_mat = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    std::vector<double> sol_vec(sol_mat.data(), sol_mat.data() + sol_mat.rows() * sol_mat.cols());
    return pcl::PointXYZ(sol_vec[0], sol_vec[1], sol_vec[2]);

}

pcl::PointXYZ model::CalibrationModel::refine_center_point(double delta) {
    if (center_point.x == 0) {
        throw std::runtime_error("Error, cannot refine because center point has not been calculated yet");
    }
    equations::Plane averaged_ground_plane = algos::average_planes(ground_planes);
    // just use the first calibration as the candidate to find a point from. in the future i can use
    // more statistical methods to determine which is the best calibration to find the best calibration to extract from
    pcl::PointXYZ plane_pt = algos::find_point_in_plane(clouds[0], averaged_ground_plane, delta);
    center_point = algos::project_point_to_plane(center_point, plane_pt, averaged_ground_plane.get_normal());
    return center_point;
}


void model::CalibrationModel::update_calibration_json() {
    file_handler.update_calibration_json(axis_of_rotation, center_point);
}


// --------------------------------------------------------------------------------
//                          PRIVATE METHODS
// --------------------------------------------------------------------------------


std::vector<equations::Plane>
model::CalibrationModel::get_calibration_planes_coefs(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                                      bool visual_flag) {

    std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> clouds;
    std::vector<equations::Plane> planes;

    auto cloud_cpy = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    *cloud_cpy = *cloud;
    auto cloud_plane = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();

    // calculate the normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_cpy);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    auto ground_coeff = std::make_shared<pcl::ModelCoefficients>();
    auto inliers = std::make_shared<pcl::PointIndices>();
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
    seg.setNormalDistanceWeight(0.02);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.005);
    seg.setInputCloud(cloud_cpy);
    seg.setInputNormals(cloud_normals);
    // set hardcoded ground normal axis value with wide epsilon value
    seg.setAxis(Eigen::Vector3f(.00295, -.7803, -.3831));
    seg.setEpsAngle(0.523599);
    seg.segment(*inliers, *ground_coeff);

    if (inliers->indices.empty()) {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
    }

    planes.emplace_back(ground_coeff);

    std::cerr << "Model coefficients: " << ground_coeff->values[0] << " "
              << ground_coeff->values[1] << " "
              << ground_coeff->values[2] << " "
              << ground_coeff->values[3] << std::endl;

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract.setInputCloud(cloud_cpy);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);

    if (visual_flag) {
        clouds = {cloud_cpy, cloud_plane};
        visual::Visualizer::simpleVis(clouds);
    }

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_cpy);

    // remove normal inliers
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers);
    extract_normals.filter(*cloud_normals);

    // LETS GET THE UPRIGHT PLANE!!

    auto up_coeff = std::make_shared<pcl::ModelCoefficients>();
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg2;

    seg2.setOptimizeCoefficients(true);
    seg2.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg2.setNormalDistanceWeight(0.02);
    seg2.setMethodType(pcl::SAC_RANSAC);
    seg2.setMaxIterations(10000);
    seg2.setDistanceThreshold(0.003);
    seg2.setInputCloud(cloud_cpy);
    seg2.setInputNormals(cloud_normals);
    seg2.segment(*inliers, *up_coeff);

    planes.emplace_back(up_coeff);

    std::cerr << "Up Model coefficients: " << up_coeff->values[0] << " "
              << up_coeff->values[1] << " "
              << up_coeff->values[2] << " "
              << up_coeff->values[3] << std::endl;

    extract.setInputCloud(cloud_cpy);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);

    if (visual_flag) {
        clouds = {cloud_cpy, cloud_plane};
        visual::Visualizer::simpleVis(clouds);
    }

    auto ground_vect = Eigen::Vector3f(ground_coeff->values[0], ground_coeff->values[1], ground_coeff->values[2]);
    auto up_vect = Eigen::Vector3f(up_coeff->values[0], up_coeff->values[1], up_coeff->values[2]);

    double angle = std::atan2(ground_vect.cross(up_vect).norm(), ground_vect.dot(up_vect));
    double angle_deg = angle * (180.0 / 3.141592653589793238463);
    std::cout << "the angle between two planes is " << angle_deg << std::endl;
    std::cout << "the error is: " << abs((angle_deg - 90) / 90.0) * 100.0 << "%" << std::endl;

    return planes;
}


Eigen::MatrixXd model::CalibrationModel::build_A_matrix(const equations::Normal &g_n,
                                                        const std::vector<equations::Plane> &upright_planes) {
    int rows = upright_planes.size() - 1;
    Eigen::MatrixXd A(rows, 3);
    for (int i = 0; i < rows; i++) {
        A(i, 0) = equations::coeff(g_n, upright_planes[i].get_normal()) * upright_planes[i].A -
                  equations::coeff(g_n, upright_planes[i + 1].get_normal()) * upright_planes[i + 1].A;
        A(i, 1) = equations::coeff(g_n, upright_planes[i].get_normal()) * upright_planes[i].B -
                  equations::coeff(g_n, upright_planes[i + 1].get_normal()) * upright_planes[i + 1].B;
        A(i, 2) = equations::coeff(g_n, upright_planes[i].get_normal()) * upright_planes[i].C -
                  equations::coeff(g_n, upright_planes[i + 1].get_normal()) * upright_planes[i + 1].C;
    }

    return A;
}

Eigen::MatrixXd model::CalibrationModel::build_b_matrix(const equations::Normal &g_n,
                                                        const std::vector<equations::Plane> &upright_planes) {
    int rows = upright_planes.size() - 1;
    Eigen::MatrixXd b(rows, 1);
    for (int i = 0; i < rows; i++) {
        b(i, 0) = equations::coeff(g_n, upright_planes[i + 1].get_normal()) * upright_planes[i + 1].D -
                  equations::coeff(g_n, upright_planes[i].get_normal()) * upright_planes[i].D;
    }
    return b;
}


equations::Normal model::CalibrationModel::calculate_axis_dir(const std::vector<equations::Plane> &ground_planes) {
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

