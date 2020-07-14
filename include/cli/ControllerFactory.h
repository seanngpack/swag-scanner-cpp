#ifndef SWAG_SCANNER_CONTROLLERFACTORY_H
#define SWAG_SCANNER_CONTROLLERFACTORY_H

#include "CalibrationController.h"
#include "ProcessingController.h"
#include "ScanController.h"

namespace cli {
    class ControllerFactory {
    public:

        std::unique_ptr<controller::CalibrationController> buildCalibrationController(std::shared_ptr<camera::ICamera> camera,
                                                                     std::shared_ptr<arduino::Arduino> arduino,
                                                                     std::shared_ptr<model::Model> model,
                                                                     std::shared_ptr<file::CalibrationFileHandler> file_handler,
                                                                     std::shared_ptr<visual::Visualizer> viewer,
                                                                     int deg,
                                                                     int num_rot);

        std::unique_ptr<controller::ProcessingController> buildProcessingController(std::shared_ptr<model::Model> model,
                                                                   std::shared_ptr<visual::Visualizer> viewer,
                                                                   std::shared_ptr<file::ScanFileHandler> file_handler);

        std::unique_ptr<controller::ScanController> buildScanController(std::shared_ptr<camera::ICamera> camera,
                                                       std::shared_ptr<arduino::Arduino> arduino,
                                                       std::shared_ptr<model::Model> model,
                                                       std::shared_ptr<file::ScanFileHandler> file_handler);

    };
}

#endif //SWAG_SCANNER_CONTROLLERFACTORY_H
