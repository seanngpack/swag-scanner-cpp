#include "ProcessingModel.h"
#include "Normal.h"
#include "Algorithms.h"
#include "Constants.h"
#include <pcl/registration/icp.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

model::ProcessingModel::ProcessingModel() :
        file_handler() {}


void model::ProcessingModel::set_scan(const std::string &scan_name) {
    file_handler.set_scan(scan_name);
    clouds = file_handler.load_clouds(CloudType::Type::FILTERED);
    // TODO: dont forget to assign cloud names to the map
}

void model::ProcessingModel::save_cloud(const std::string &cloud_name, const CloudType::Type &cloud_type) {
    auto cloud = clouds[clouds_map[cloud_name]];
    file_handler.save_cloud(cloud, cloud_name, cloud_type);
}

void model::ProcessingModel::save_cloud(
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud,
        const std::string &cloud_name,
        const CloudType::Type &cloud_type) {
    file_handler.save_cloud(cloud, cloud_name, cloud_type);
}

void model::ProcessingModel::filter(int mean_k,
                                    float thresh_mult) {
    using namespace constants;
    for (int i = 0; i < clouds.size(); i++) {
        crop_cloud(clouds[i],
                   scan_min_x, scan_max_x,
                   scan_min_y, scan_max_y,
                   scan_min_z, scan_max_z);
        remove_nan(clouds[i]);
        remove_outliers(clouds[i]);
        // do cloud saving here, try to get the name from the map
        save_cloud(clouds[i], std::to_string(i) + ".pcd", CloudType::Type::PROCESSED);
    }
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
    save_cloud(global_cloud, "REGISTERED.pcd", CloudType::Type::PROCESSED);
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
    std::cout << "registering clouds..." << std::endl;
    icp.align(*transformed_cloud);
    if (icp.hasConverged()) {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
        auto trans = icp.getFinalTransformation().cast<double>();
        std::cout << trans << std::endl;
    } else {
        PCL_ERROR ("\nICP has not converged.\n");
    }
    return icp.getFinalTransformation();
}

