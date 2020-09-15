#include "ProcessingController.h"
#include "Visualizer.h"
#include "ScanFileHandler.h"
#include "Model.h"
#include "Constants.h"
#include <pcl/common/transforms.h>
#include <memory>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

controller::ProcessingController::ProcessingController(std::shared_ptr<model::Model> model,
                                                       std::shared_ptr<visual::Visualizer> viewer,
                                                       std::shared_ptr<file::ScanFileHandler> file_handler) :
        model(std::move(model)), viewer(std::move(viewer)), file_handler(std::move(file_handler)) {}

void controller::ProcessingController::run() {
    filter(CloudType::Type::FILTERED);
}

void controller::ProcessingController::filter(const CloudType::Type &cloud_type) {
    using namespace constants;

    /**
     * Perform cropping
     */
    std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> clouds = file_handler->load_clouds(cloud_type);
    for (int i = 0; i < clouds.size(); i++) {
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> filtered_cloud = model->crop_cloud(clouds[i],
                                                                                           cal_min_x, cal_max_x,
                                                                                           cal_min_y, cal_max_y,
                                                                                           cal_min_z, cal_max_z);
        filtered_cloud = model->remove_nan(filtered_cloud);
        filtered_cloud = model->remove_outliers(filtered_cloud);
        file_handler->save_cloud(filtered_cloud, std::to_string(i), CloudType::Type::PROCESSED); // remove this later
    }
}


void controller::ProcessingController::register_all_clouds(const CloudType::Type &cloud_type) {
    std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> cloud_vector = file_handler->load_clouds(cloud_type);
    Eigen::Matrix4f global_transform = Eigen::Matrix4f::Identity();
    auto source = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto target = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto result = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto global_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    for (int i = 1; i < cloud_vector.size(); i++) {

        source = cloud_vector[i - 1];
        target = cloud_vector[i];
        auto temp_registered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        Eigen::Matrix4f transform = model->icp_register_pair_clouds(source, target, temp_registered);
        Eigen::Matrix4f targetToSource = transform.inverse();

        pcl::transformPointCloud(*temp_registered, *result, global_transform);

//        file_handler->save_cloud(result, std::to_string(i), CloudType::Type::NORMAL);
        *global_cloud += *result;
        global_transform *= targetToSource;

    }

    viewer->simpleVis(global_cloud);
}


