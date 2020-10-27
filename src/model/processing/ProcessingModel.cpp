#include "ProcessingModel.h"
#include "Normal.h"
#include "Algorithms.h"
#include "Constants.h"
#include "Logger.h"
#include <pcl/registration/icp.h>
#include <pcl/surface/gp3.h>
#include <pcl/PolygonMesh.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

model::ProcessingModel::ProcessingModel() : file_handler() {}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> model::ProcessingModel::load_cloud(const std::string &name,
                                                                                   const CloudType::Type type) {
    return file_handler.load_cloud(name, type);
}

void model::ProcessingModel::set_scan(const std::string &scan_name) {
    file_handler.set_scan(scan_name);
    // TODO: probably decouple loading clouds from this method
    clouds = file_handler.load_clouds(CloudType::Type::RAW);
    // TODO: dont forget to assign cloud names to the map
}

void model::ProcessingModel::save_cloud(const std::string &cloud_name, const CloudType::Type &cloud_type) {
    auto cloud = clouds[clouds_map[cloud_name]];
    file_handler.save_cloud(cloud, cloud_name, cloud_type);
}

void model::ProcessingModel::save_cloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud,
                                        const std::string &cloud_name,
                                        const CloudType::Type &cloud_type) {
    file_handler.save_cloud(cloud, cloud_name, cloud_type);
}

void model::ProcessingModel::filter(int sigma_s,
                                    float sigma_r,
                                    int mean_k,
                                    float thresh_mult) {
    using namespace constants;
    int clouds_vector_size = clouds.size();
    for (int i = 0; i < clouds_vector_size; i++) {
        crop_cloud(clouds[i],
                   scan_min_x, scan_max_x,
                   scan_min_y, scan_max_y,
                   scan_min_z, scan_max_z);
        bilateral_filter(clouds[i], sigma_s, sigma_r);
        remove_nan(clouds[i]);
        remove_outliers(clouds[i]);
        // do cloud saving here, try to get the name from the map
        add_cloud(clouds[i], std::to_string(i) + ".pcd");
        save_cloud(clouds[i], std::to_string(i) + ".pcd", CloudType::Type::FILTERED);
    }
}

void model::ProcessingModel::mesh() {
    auto cloud_with_normals = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    auto registered = load_cloud("REGISTERED.pcd", CloudType::Type::REGISTERED);
    auto normals = calculate_normals(registered);

    // Concatenate the XYZ and normal fields*
    pcl::concatenateFields(*registered, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointNormal>>();
    tree->setInputCloud(cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    auto triangles = std::make_shared<pcl::PolygonMesh>();

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(0.025);

    // Set typical values for the parameters
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18); // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree);
    gp3.reconstruct(*triangles);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    file_handler.save_mesh(triangles, "mesh.obj", CloudType::Type::MESH);
}

void model::ProcessingModel::transform_clouds_to_world() {
    if (clouds.empty()) {
        throw std::runtime_error("Cannot perform transformation, must load clouds first.");
    }
    json calibration_json = file_handler.get_calibration_json();
    std::vector<double> temp0 = calibration_json["axis_direction"].get<std::vector<double>>();
    equations::Normal rot_axis(temp0);
    auto temp = calibration_json["origin_point"].get<std::vector<double>>();
    pcl::PointXYZ center_pt(temp[0], temp[1], temp[2]);

    for (auto &cloud: clouds) {
        Eigen::Matrix4f transform = algos::calc_transform_to_world_matrix(center_pt, rot_axis);
        pcl::transformPointCloud(*cloud, *cloud, transform);
    }
}

void model::ProcessingModel::register_clouds() {
    json info_json = file_handler.get_info_json();
    int angle = info_json["angle"];

    auto global_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    *global_cloud = *clouds[0];
    pcl::PointCloud<pcl::PointXYZ> rotated;
    for (int i = 1; i < clouds.size(); i++) {
        rotated = algos::rotate_cloud_about_z_axis(clouds[i], angle * i);
        *global_cloud += rotated;
    }
    remove_outliers(global_cloud, 50, 1);
    add_cloud(global_cloud, "REGISTERED.pcd");
    save_cloud(global_cloud, "REGISTERED.pcd", CloudType::Type::REGISTERED);
}

Eigen::Matrix4f
model::ProcessingModel::icp_register_pair_clouds(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud_src,
                                                 const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud_target,
                                                 std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &transformed_cloud) {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_src);
    icp.setInputTarget(cloud_target);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-10);
    icp.setMaxCorrespondenceDistance(.05); // not really sure how this affects results
    icp.setEuclideanFitnessEpsilon(.0001); // big effect
    icp.setRANSACOutlierRejectionThreshold(.0001); // doesn't seem to affect results much
    logger::info("ICP registering clouds...");
    icp.align(*transformed_cloud);
    if (icp.hasConverged()) {
        logger::info("ICP has converged, score is: " + std::to_string(icp.getFitnessScore()));
        auto trans = icp.getFinalTransformation().cast<double>();
        std::stringstream ss;
        ss << trans;
        logger::info(ss.str());
    } else {
        PCL_ERROR ("\nICP has not converged.\n");
    }
    return icp.getFinalTransformation();
}

