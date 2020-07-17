#include <CalibrationController.h>
#include <utility>

controller::CalibrationController::CalibrationController(std::shared_ptr<camera::ICamera> camera,
                                                         std::shared_ptr<arduino::Arduino> arduino,
                                                         std::shared_ptr<model::Model> model,
                                                         std::shared_ptr<file::CalibrationFileHandler> file_handler,
                                                         std::shared_ptr<visual::Visualizer> viewer,
                                                         int deg,
                                                         int num_rot) :
        camera(std::move(camera)), arduino(std::move(arduino)), model(std::move(model)),
        file_handler(std::move(file_handler)),
        viewer(std::move(viewer)), deg(deg), num_rot(num_rot) {}

void controller::CalibrationController::run() {
    scan();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr>> cloud_vector;
    file_handler->load_clouds(cloud_vector, CloudType::Type::CALIBRATION);

    std::vector<equations::Plane> ground_planes;
    std::vector<equations::Plane> upright_planes;
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud : cloud_vector) {
        std::vector<equations::Plane> coeffs = model->get_calibration_planes_coefs(cloud);
        upright_planes.push_back(coeffs[0]);
        ground_planes.push_back(coeffs[1]);
    }
    equations::Normal axis_dir = model->calculate_axis_dir(ground_planes);
    equations::Point center = model->calculate_center_pt(axis_dir, upright_planes);
    file_handler->update_calibration_json(axis_dir, center);
}

void controller::CalibrationController::scan() {
    const camera::ss_intrinsics *intrin = camera->get_intrinsics();
    std::cout << "starting scanning..." << std::endl;
    for (int i = 0; i < num_rot; i++) {
        std::string name = std::to_string(i * deg) + ".pcd";
        std::vector<uint16_t> depth_frame = camera->get_depth_frame();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = model->create_point_cloud(depth_frame, intrin);
        file_handler->save_cloud(cloud, name, CloudType::Type::CALIBRATION);
        arduino->rotate_table(deg);
    }
}


