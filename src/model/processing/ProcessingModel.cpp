#include "ProcessingModel.h"
#include "Normal.h"
#include "Algorithms.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

model::ProcessingModel::ProcessingModel() :
        file_handler() {}


void model::ProcessingModel::set_scan(const std::string &scan_name) {
    file_handler.set_scan(scan_name);
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
    for (auto &cloud: clouds) {
        remove_nan(cloud);
        remove_outliers(cloud);
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

