#include "ControllerFactory.h"
#include "CalibrationController.h"
#include "ProcessingController.h"
#include "ScanController.h"

namespace po = boost::program_options;

std::unique_ptr<controller::IController> cli::ControllerFactory::create(po::variables_map vm) {
    if (vm.count("scan")) {
        return create_scan_controller(vm);
    } else if (vm.count("calibrate")) {
        return create_calibrate_controller(vm);
    } else if (vm.count("process")) {
        return create_processing_controller(vm);
    }
}

std::unique_ptr<controller::IController>
cli::ControllerFactory::create_scan_controller(boost::program_options::variables_map vm) {
    std::shared_ptr<file::ScanFileHandler> file_handler;
    int deg = 20;
    int num_rot = 18;
    if (vm.count("name")) {
        const char *c = vm["name"].as<std::string>().c_str();
        file_handler = std::make_shared<file::ScanFileHandler>(c);
    } else {
        file_handler = std::make_shared<file::ScanFileHandler>(true);
    }
    if (vm.count("deg")) {
        deg = vm["deg"].as<int>();
    }
    if (vm.count("rot")) {
        num_rot = vm["rot"].as<int>();
    }
    return std::make_unique<controller::ScanController>(std::make_shared<camera::SR305>(),
                                                        std::make_shared<arduino::Arduino>(),
                                                        std::make_shared<model::Model>(),
                                                        file_handler,
                                                        deg,
                                                        num_rot);
}

std::unique_ptr<controller::IController>
cli::ControllerFactory::create_calibrate_controller(boost::program_options::variables_map vm) {
    std::shared_ptr<file::CalibrationFileHandler> file_handler;
    int deg = 15;
    int num_rot = 7;
    if (vm.count("name")) {
        const char *c = vm["name"].as<std::string>().c_str();
        file_handler = std::make_shared<file::CalibrationFileHandler>(c);
    } else {
        file_handler = std::make_shared<file::CalibrationFileHandler>();
    }
    if (vm.count("deg")) {
        deg = vm["deg"].as<int>();
    }
    if (vm.count("rot")) {
        num_rot = vm["rot"].as<int>();
    }
    return std::make_unique<controller::CalibrationController>(std::make_shared<camera::SR305>(),
                                                               std::make_shared<arduino::Arduino>(),
                                                               std::make_shared<model::Model>(),
                                                               file_handler,
                                                               std::make_shared<visual::Visualizer>(),
                                                               deg,
                                                               num_rot);
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









