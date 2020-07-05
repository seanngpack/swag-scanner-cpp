#include <CalibrationController.h>

controller::CalibrationController::CalibrationController(camera::ICamera *camera,
                                                         arduino::Arduino *arduino,
                                                         std::shared_ptr<model::Model> model,
                                                         std::shared_ptr<file::FileHandler> file_handler,
                                                         visual::Visualizer *viewer) :
        camera(camera), arduino(arduino), model(model), file_handler(file_handler), viewer(viewer) {}

void controller::CalibrationController::run() {

    // don forget to make new calibration.json
}

void controller::CalibrationController::scan(int deg, int n) {
    if (360 % deg != 0) {
        throw std::invalid_argument("Invalid input, scanning interval must be a factor of 360");
    }

    const camera::ss_intrinsics *intrin = camera->get_intrinsics();
    std::cout << "starting scanning..." << std::endl;
    for (int i = 0; i < n; i++) {
        std::string name = std::to_string(i * deg);
        const uint16_t *depth_frame = camera->get_depth_frame();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = model->create_point_cloud(depth_frame, intrin);
        file_handler->save_cloud(cloud, name, CloudType::Type::CALIBRATION);
        arduino->rotate_table(deg);
    }

}

controller::CalibrationController::~CalibrationController() {
    delete viewer;
}

