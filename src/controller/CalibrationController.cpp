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
    file_handler->load_clouds()

    calibration::
    // don forget to make new calibration.json
}


controller::CalibrationController::~CalibrationController() {
    delete viewer;
}

