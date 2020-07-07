#include <CalibrationController.h>

controller::CalibrationController::CalibrationController(std::unique_ptr<controller::ScanController> scan_controller,
                                                         std::shared_ptr<model::Model> model,
                                                         std::shared_ptr<file::FileHandler> file_handler,
                                                         visual::Visualizer *viewer,
                                                         int deg,
                                                         int num_rot) :
        scan_controller(std::move(scan_controller)), model(model), file_handler(file_handler), viewer(viewer),
        deg(deg), num_rot(num_rot) {}

void controller::CalibrationController::run() {
    scan_controller->scan(deg, num_rot, CloudType::Type::CALIBRATION);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr>> cloud_vector;
    file_handler->load_clouds(cloud_vector, CloudType::Type::CALIBRATION);

    std::vector<equations::Plane> ground_planes;
    std::vector<equations::Plane> upright_planes;
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud : cloud_vector) {
        std::vector<equations::Plane> coeffs = model->get_calibration_planes_coefs(cloud);
        upright_planes.push_back(coeffs[0]);
        ground_planes.push_back(coeffs[1]);
    }
    equations::Point center = model->calculate_center_pt(ground_planes, upright_planes);
    // update calibration.json inside the new calibration folder
//    calibration::
}


controller::CalibrationController::~CalibrationController() {
    delete viewer;
}

