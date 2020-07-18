#include "ControllerFactory.h"
#include "CalibrationController.h"
#include "ProcessingController.h"
#include "ScanController.h"
#include "FilterTestingController.h"

namespace po = boost::program_options;

std::unique_ptr<controller::IController> cli::ControllerFactory::create(po::variables_map vm) {
    if (vm.count("scan")) {
        return create_scan_controller(vm);
    } else if (vm.count("calibrate")) {
        return create_calibrate_controller(vm);
    } else if (vm.count("process")) {
        return create_processing_controller(vm);
    } else if (vm.count("filter_test")) {
        return create_filter_testing_controller(vm);
    } else {
        throw std::invalid_argument("Error, must enter a valid base command.");
    }
}

std::unique_ptr<controller::IController>
cli::ControllerFactory::create_scan_controller(boost::program_options::variables_map vm) {
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
cli::ControllerFactory::create_calibrate_controller(boost::program_options::variables_map vm) {
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
cli::ControllerFactory::create_processing_controller(boost::program_options::variables_map vm) {
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
cli::ControllerFactory::create_filter_testing_controller(boost::program_options::variables_map vm) {
    std::shared_ptr<camera::SR305> camera = std::make_shared<camera::SR305>();
    if (vm.count("d_mag")) {
        camera->set_decimation_magnitude(vm["d_mag"].as<int>());
    }
    if (vm.count("s_mag")) {
        camera->set_decimation_magnitude(vm["s_mag"].as<int>());
    }
    if (vm.count("s_alpha")) {
        camera->set_decimation_magnitude(vm["s_alpha"].as<float>());
    }
    if (vm.count("s_delta")) {
        camera->set_decimation_magnitude(vm["s_delta"].as<int>());
    }
    if (vm.count("t_alpha")) {
        camera->set_decimation_magnitude(vm["t_alpha"].as<float>());
    }
    if (vm.count("t_delta")) {
        camera->set_decimation_magnitude(vm["t_delta"].as<int>());
    }
    if (vm.count("t_persis")) {
        camera->set_decimation_magnitude(vm["t_persis"].as<int>());
    }
    return std::make_unique<controller::FilterTestingController>(camera,
                                                                 std::make_shared<model::Model>(),
                                                                 std::make_shared<file::ScanFileHandler>(),
                                                                 std::make_shared<visual::Visualizer>());
}









