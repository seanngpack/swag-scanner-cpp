#include <CalibrationController.h>
#include <utility>
#include "Constants.h"

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
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr>> cloud_vector;
    file_handler->load_clouds(cloud_vector, CloudType::Type::CALIBRATION);
    equations::Normal axis_dir = model->calculate_axis_dir(ground_planes);
    equations::Point center = model->calculate_center_pt(axis_dir, upright_planes);
    file_handler->update_calibration_json(axis_dir, center);
    arduino->rotate_to(0);

    viewer->ptVis(cloud_vector[0], pcl::PointXYZ(center.x, center.y, center.z));
}

void controller::CalibrationController::scan() {
    using namespace constants;

    camera->scan();
    const camera::ss_intrinsics intrin = camera->get_intrinsics_processed();
    std::cout << "starting scanning..." << std::endl;
    for (int i = 0; i < num_rot; i++) {
        std::string name = std::to_string(i * deg) + ".pcd";
        camera->scan();
        std::vector<uint16_t> depth_frame = camera->get_depth_frame_processed();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = model->create_point_cloud(depth_frame, intrin);
        cloud = model->crop_cloud(cloud, min_x, max_x, min_y, max_y, min_z, max_z);
        cloud = model->voxel_grid_filter(cloud, .003);

        std::vector<equations::Plane> coeffs = model->get_calibration_planes_coefs(cloud);
        upright_planes.emplace_back(coeffs[0]);
        ground_planes.emplace_back(coeffs[1]);

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


