#include "ControllerFactory.h"
#include "IController.h"
#include "CalibrationController.h"
#include "ProcessingController.h"
#include "ScanController.h"
#include "FilterTestingController.h"
#include "MoveController.h"
#include "HomeController.h"

#include "CalibrationFileHandler.h"
#include "ScanFileHandler.h"

#include "SR305.h"
#include "Arduino.h"
#include "Model.h"
#include "Visualizer.h"

namespace po = boost::program_options;

controller::ControllerFactory::ControllerFactory() : cache(std::make_unique<ControllerFactoryCache>()) {}

std::shared_ptr<controller::IController> controller::ControllerFactory::create(const po::variables_map &vm) {
    if (vm.count("scan")) {
        return create_scan_controller(vm);
    } else if (vm.count("calibrate")) {
        return create_calibrate_controller(vm);
    } else if (vm.count("process")) {
        return create_processing_controller(vm);
    } else if (vm.count("filter_test")) {
        return create_filter_testing_controller(vm);
    } else if (vm.count("move")) {
        return create_move_controller(vm);
    } else if (vm.count("set_home")) {
        return create_home_controller(vm);
    } else {
        throw std::invalid_argument("Error, must enter a valid base command.");
    }
}

std::shared_ptr<controller::IController>
controller::ControllerFactory::create_scan_controller(const po::variables_map &vm) {
    return cache->get_scan_controller(vm);
}

std::shared_ptr<controller::IController>
controller::ControllerFactory::create_calibrate_controller(const po::variables_map &vm) {
return cache->get_calibration_controller(vm);
}

std::shared_ptr<controller::IController>
controller::ControllerFactory::create_processing_controller(const po::variables_map &vm) {
   return cache->get_process_controller(vm);
}

std::shared_ptr<controller::IController>
controller::ControllerFactory::create_filter_testing_controller(const po::variables_map &vm) {
    std::shared_ptr<camera::SR305> camera = std::make_shared<camera::SR305>();
    if (vm.count("d_mag")) {
        camera->set_decimation_magnitude(vm["d_mag"].as<int>());
    }
    if (vm.count("s_mag")) {
        camera->set_spatial_filter_magnitude(vm["s_mag"].as<int>());
    }
    if (vm.count("s_alpha")) {
        camera->set_spatial_smooth_alpha(vm["s_alpha"].as<float>());
    }
    if (vm.count("s_delta")) {
        camera->set_spatial_smooth_delta(vm["s_delta"].as<int>());
    }
    return std::make_unique<controller::FilterTestingController>(camera,
                                                                 std::make_shared<model::Model>(),
                                                                 std::make_shared<file::ScanFileHandler>(),
                                                                 std::make_shared<visual::Visualizer>());
}

std::shared_ptr<controller::IController>
controller::ControllerFactory::create_move_controller(const po::variables_map &vm) {
    auto arduino = std::make_shared<arduino::Arduino>();
    std::shared_ptr<controller::MoveController> move_controller = std::make_unique<controller::MoveController>(arduino);
    if (vm.count("to")) {
        move_controller->set_deg(vm["to"].as<int>());
        move_controller->set_move_method("to");
    } else if (vm.count("by")) {
        move_controller->set_deg(vm["by"].as<int>());
        move_controller->set_move_method("by");
    } else if (vm.count("home")) {
        move_controller->set_deg(0);
        move_controller->set_move_method("to");
    }
    return move_controller;
}

std::shared_ptr<controller::IController>
controller::ControllerFactory::create_home_controller(const po::variables_map &vm) {
    return std::make_unique<controller::HomeController>();
}

