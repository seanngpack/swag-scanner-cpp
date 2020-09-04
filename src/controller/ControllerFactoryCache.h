#ifndef SWAG_SCANNER_CONTROLLERFACTORYCACHE_H
#define SWAG_SCANNER_CONTROLLERFACTORYCACHE_H

#include "SR305.h"
#include "Arduino.h"
#include "Model.h"
#include "Visualizer.h"
#include "CalibrationFileHandler.h"
#include "CalibrationController.h"
#include "ProcessingController.h"
#include "ScanController.h"
#include "MoveController.h"
#include "FilterTestingController.h"
#include "HomeController.h"
#include "MoveController.h"
#include <memory>
#include <ScanFileHandler.h>
#include <boost/program_options.hpp>


namespace controller {
    /**
     * This manages objects needed to create controllers.
     */
    class ControllerFactoryCache {
    public:
        /**
         * Default constructor preallocates expensive objects that are probably gonna be used.
         * Preallocated objects: Model, ScanFileHandler, CalibrationFileHandler
         */
        ControllerFactoryCache();

        std::shared_ptr<camera::SR305> get_camera();

        std::shared_ptr<arduino::Arduino> get_arduino();

        std::shared_ptr<model::Model> get_model();

        std::shared_ptr<visual::Visualizer> get_viewer();

        std::shared_ptr<file::ScanFileHandler> get_scan_file_handler();

        std::shared_ptr<file::CalibrationFileHandler> get_calibration_file_handler();

        // Controllers

        std::shared_ptr<controller::ScanController>
        get_scan_controller(const boost::program_options::variables_map &vm);

        std::shared_ptr<controller::ScanController> get_scan_controller();


        std::shared_ptr<controller::CalibrationController>
        get_calibration_controller(const boost::program_options::variables_map &vm);

        std::shared_ptr<controller::CalibrationController> get_calibration_controller();

        std::shared_ptr<controller::ProcessingController>
        get_process_controller(const boost::program_options::variables_map &vm);

        std::shared_ptr<controller::ProcessingController> get_process_controller();

        std::shared_ptr<controller::FilterTestingController>
        get_filter_testing_controller(const boost::program_options::variables_map &vm);

        std::shared_ptr<controller::MoveController>
        get_move_controller(const boost::program_options::variables_map &vm);

        std::shared_ptr<controller::HomeController>
        get_home_controller(const boost::program_options::variables_map &vm);

    private:
        std::shared_ptr<camera::SR305> camera;
        std::shared_ptr<arduino::Arduino> arduino;
        std::shared_ptr<model::Model> model;
        std::shared_ptr<visual::Visualizer> viewer;
        std::shared_ptr<file::ScanFileHandler> scan_file_handler;
        std::shared_ptr<file::CalibrationFileHandler> calibration_file_handler;

        std::shared_ptr<controller::ScanController> scan_controller;
        std::shared_ptr<controller::CalibrationController> calibration_controller;
        std::shared_ptr<controller::ProcessingController> process_controller;

        // controllers

    };
}

#endif //SWAG_SCANNER_CONTROLLERFACTORYCACHE_H
