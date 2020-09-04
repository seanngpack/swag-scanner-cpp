#include "ControllerFactory.h"
#include "IController.h"
#include "calibration/CalibrationController.h"
#include "processing/ProcessingController.h"
#include "scan/ScanController.h"
#include "filter_testing/FilterTestingController.h"
#include "move/MoveController.h"
#include "home/HomeController.h"

#include "CalibrationFileHandler.h"
#include "ScanFileHandler.h"

#include "SR305.h"
#include "Arduino.h"
#include "Model.h"
#include "Visualizer.h"

namespace po = boost::program_options;

std::unique_ptr<controller::IController> controller::ControllerFactory::create(po::variables_map vm) {
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

std::unique_ptr<controller::IController>
controller::ControllerFactory::create_scan_controller(boost::program_options::variables_map vm) {
    std::shared_ptr<file::ScanFileHandler> file_handler;
    if (vm.count("name")) {
        const char *c = vm["name"].as<std::string>().c_str();
        file_handler = std::make_shared<file::ScanFileHandler>(c);
    } else {
        file_handler = std::make_shared<file::ScanFileHandler>(true);
    }

    std::unique_ptr<controller::ScanController> controller = std::make_unique<controller::ScanController>(
            std::make_shared<camera::SR305>(),
            std::make_shared<arduino::Arduino>(),
            std::make_shared<model::Model>(),
            file_handler);

    if (vm.count("deg")) {
        controller->set_deg(vm["deg"].as<int>());
    }
    if (vm.count("rot")) {
        controller->set_num_rot(vm["rot"].as<int>());
    }
    return controller;
}

std::unique_ptr<controller::IController>
controller::ControllerFactory::create_calibrate_controller(boost::program_options::variables_map vm) {
    std::shared_ptr<file::CalibrationFileHandler> file_handler;
    if (vm.count("name")) {
        const char *c = vm["name"].as<std::string>().c_str();
        file_handler = std::make_shared<file::CalibrationFileHandler>(c);
    } else {
        file_handler = std::make_shared<file::CalibrationFileHandler>();
    }

    std::unique_ptr<controller::CalibrationController> controller = std::make_unique<controller::CalibrationController>(
            std::make_shared<camera::SR305>(),
            std::make_shared<arduino::Arduino>(),
            std::make_shared<model::Model>(),
            file_handler,
            std::make_shared<visual::Visualizer>());

    if (vm.count("deg")) {
        controller->set_deg(vm["deg"].as<int>());
    }
    if (vm.count("rot")) {
        controller->set_num_rot(vm["rot"].as<int>());
    }
    return controller;
}

std::unique_ptr<controller::IController>
controller::ControllerFactory::create_processing_controller(boost::program_options::variables_map vm) {
    std::shared_ptr<file::ScanFileHandler> file_handler;
    if (vm.count("name")) {
        const char *c = vm["name"].as<std::string>().c_str();
        file_handler = std::make_shared<file::ScanFileHandler>(c);
    } else {
        file_handler = std::make_shared<file::ScanFileHandler>();
    }
    return std::make_unique<controller::ProcessingController>(std::make_shared<model::Model>(),
                                                              std::make_shared<visual::Visualizer>(),
                                                              file_handler);
}

std::unique_ptr<controller::IController>
controller::ControllerFactory::create_filter_testing_controller(boost::program_options::variables_map vm) {
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

std::unique_ptr<controller::IController>
controller::ControllerFactory::create_move_controller(boost::program_options::variables_map vm) {
    auto arduino = std::make_shared<arduino::Arduino>();
    std::unique_ptr<controller::MoveController> move_controller = std::make_unique<controller::MoveController>(arduino);
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

std::unique_ptr<controller::IController>
controller::ControllerFactory::create_home_controller(boost::program_options::variables_map vm) {
    return std::make_unique<controller::HomeController>();
}









