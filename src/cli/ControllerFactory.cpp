#include "ControllerFactory.h"

std::unique_ptr<controller::CalibrationController>
cli::ControllerFactory::buildCalibrationController(std::shared_ptr<camera::ICamera> camera,
                                                   std::shared_ptr<arduino::Arduino> arduino,
                                                   std::shared_ptr<model::Model> model,
                                                   std::shared_ptr<file::CalibrationFileHandler> file_handler,
                                                   std::shared_ptr<visual::Visualizer> viewer, int deg, int num_rot) {
    return std::make_unique<controller::CalibrationController>(camera, arduino, model, file_handler, viewer, deg,
                                                               num_rot);
}

std::unique_ptr<controller::ProcessingController>
cli::ControllerFactory::buildProcessingController(std::shared_ptr<model::Model> model,
                                                  std::shared_ptr<visual::Visualizer> viewer,
                                                  std::shared_ptr<file::ScanFileHandler> file_handler) {
    return std::make_unique<controller::ProcessingController>(model, viewer, file_handler);
}


std::unique_ptr<controller::ScanController>
cli::ControllerFactory::buildScanController(std::shared_ptr<camera::ICamera> camera,
                                            std::shared_ptr<arduino::Arduino> arduino,
                                            std::shared_ptr<model::Model> model,
                                            std::shared_ptr<file::ScanFileHandler> file_handler) {
    return std::make_unique<controller::ScanController>(camera, arduino, model, file_handler);
}




