#ifndef SWAG_SCANNER_CONTROLLERFACTORYCACHE_H
#define SWAG_SCANNER_CONTROLLERFACTORYCACHE_H

#include <SR305.h>
#include <Arduino.h>
#include <Model.h>
#include <Visualizer.h>
#include <memory>
#include <ScanFileHandler.h>
#include <CalibrationFileHandler.h>

namespace controller {
    /**
     * This manages objects needed to create controllers.
     */
    class ControllerFactoryCache {
    public:
        std::shared_ptr<camera::SR305> get_camera();

        std::shared_ptr<arduino::Arduino> get_arduino();

        std::shared_ptr<model::Model> get_model();

        std::shared_ptr<visual::Visualizer> get_viewer();

        std::shared_ptr<file::ScanFileHandler> get_scan_file_handler();

        std::shared_ptr<file::CalibrationFileHandler> get_calibration_file_handler();

    private:
        std::shared_ptr<camera::SR305> camera;
        std::shared_ptr<arduino::Arduino> arduino;
        std::shared_ptr<model::Model> model;
        std::shared_ptr<visual::Visualizer> viewer;
        std::shared_ptr<file::ScanFileHandler> scan_file_handler;
        std::shared_ptr<file::CalibrationFileHandler> calibration_file_handler;
    };
}

#endif //SWAG_SCANNER_CONTROLLERFACTORYCACHE_H
