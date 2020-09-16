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
    get_calibration_planes();
    calculate();
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
        cloud = model->crop_cloud(cloud, cal_min_x, cal_max_x, cal_min_y, cal_max_y, cal_min_z, cal_max_z);
        cloud = model->voxel_grid_filter(cloud, .001);
        file_handler->save_cloud(cloud, name, CloudType::Type::CALIBRATION);
        arduino->rotate_by(deg);
    }
    arduino->rotate_to(0);

}

void controller::CalibrationController::set_deg(int deg) {
    this->deg = deg;
}

void controller::CalibrationController::set_num_rot(int num_rot) {
    this->num_rot = num_rot;
}

void controller::CalibrationController::get_calibration_planes() {
    clouds = file_handler->load_clouds(CloudType::Type::CALIBRATION);
    for (const auto &c : clouds) {
        std::vector<equations::Plane> coeffs = model->get_calibration_planes_coefs(c);
        ground_planes.emplace_back(coeffs[0]);
        upright_planes.emplace_back(coeffs[1]);
    }
}

void controller::CalibrationController::calculate() {
    equations::Normal axis_dir = model->calculate_axis_dir(ground_planes);
    pcl::PointXYZ center = model->calculate_center_pt(axis_dir, upright_planes);
    equations::Plane averaged_ground_plane = model->average_planes(ground_planes);
    pcl::PointXYZ refined_center = model->refine_center_pt(clouds[0],
                                                           center,
                                                           averaged_ground_plane);
    file_handler->update_calibration_json(axis_dir, refined_center);

    clouds.clear();
}


