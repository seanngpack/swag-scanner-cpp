#include "CalibrationController.h"
#include "CalibrationFileHandler.h"
#include "Model.h"
#include "Constants.h"
#include "Normal.h"
#include "Plane.h"
#include "Point.h"
#include "SR305.h"
#include "Arduino.h"
#include "Visualizer.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>


controller::CalibrationController::CalibrationController(std::shared_ptr<camera::ICamera> camera,
                                                         std::shared_ptr<arduino::Arduino> arduino,
                                                         std::shared_ptr<model::Model> model,
                                                         std::shared_ptr<file::CalibrationFileHandler> file_handler,
                                                         std::shared_ptr<visual::Visualizer> viewer) :
        camera(std::move(camera)), arduino(std::move(arduino)), model(std::move(model)),
        file_handler(std::move(file_handler)),
        viewer(std::move(viewer)) {}

void controller::CalibrationController::run() {
    scan();
    std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> cloud_vector = file_handler->load_clouds(
            CloudType::Type::CALIBRATION);
    equations::Normal axis_dir = model->calculate_axis_dir(ground_planes);
    equations::Point center = model->calculate_center_pt(axis_dir, upright_planes);
    file_handler->update_calibration_json(axis_dir, center);
    arduino->rotate_to(0);

//    viewer->ptVis(cloud_vector[0], pcl::PointXYZ(center.x, center.y, center.z));
}

void controller::CalibrationController::scan() {
    using namespace constants;

    const camera::intrinsics intrin = camera->get_intrinsics_processed();
    for (int i = 0; i < num_rot; i++) {
        std::string name = std::to_string(i * deg) + ".pcd";
        camera->scan();
        std::vector<uint16_t> depth_frame = camera->get_depth_frame_processed();
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud = model->create_point_cloud(depth_frame, intrin);
        cloud = model->crop_cloud(cloud, min_x, max_x, min_y, max_y, min_z, max_z);
        cloud = model->voxel_grid_filter(cloud, .003);

        std::vector<equations::Plane> coeffs = model->get_calibration_planes_coefs(cloud);
        ground_planes.emplace_back(coeffs[0]);
        upright_planes.emplace_back(coeffs[1]);

        file_handler->save_cloud(cloud, name, CloudType::Type::CALIBRATION);
        arduino->rotate_by(deg);
    }
}

void controller::CalibrationController::set_deg(int deg) {
    this->deg = deg;
}

void controller::CalibrationController::set_num_rot(int num_rot) {
    this->num_rot = num_rot;
}


