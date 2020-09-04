#include "ControllerFactory.h"
#include "IController.h"
#include "IControllerGUI.h"
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
#include "CalibrationControllerGUI.h"
#include "HomeController.h"
#include "ControllerFactoryCache.h"
#include "MoveController.h"


namespace po = boost::program_options;

controller::ControllerFactory::ControllerFactory() : cache(std::make_unique<ControllerFactoryCache>(this)) {}

std::shared_ptr<controller::IController> controller::ControllerFactory::get_controller(const po::variables_map &vm) {
    if (vm.count("scan")) {
        return cache->get_scan_controller(vm);
    } else if (vm.count("calibrate")) {
        return cache->get_calibration_controller(vm);
    } else if (vm.count("process")) {
        return cache->get_process_controller(vm);
    } else if (vm.count("filter_test")) {
        return cache->get_filter_testing_controller(vm);
    } else if (vm.count("move")) {
        return cache->get_move_controller(vm);
    } else if (vm.count("set_home")) {
        return cache->get_home_controller(vm);
    } else {
        throw std::invalid_argument("Error, must enter a valid base command.");
    }
}

std::shared_ptr<controller::IController> controller::ControllerFactory::get_controller(const std::string &name) {
    if (name == "scan") {
        return cache->get_scan_controller();
    } else if (name == "calibrate") {
        return cache->get_calibration_controller();
    } else if (name == "process") {
        return cache->get_process_controller();
    } else {
        throw std::invalid_argument("Error, must enter a valid base command.");
    }
}

std::shared_ptr<controller::IControllerGUI> controller::ControllerFactory::get_gui_controller(const std::string &name) {
    if (name == "calibrate") {
        std::cout << "calling cache to retreive controller" << std::endl;
        return cache->get_calibration_controller_gui();
    } else {
        throw std::invalid_argument("Error, must enter a valid controller name.");
    }
//    if (name == "scan") {
//        return cache->get_scan_controller();
//    } else if (name == "calibrate") {
//        return cache->get_calibration_controller();
//    } else if (name == "process") {
//        return cache->get_process_controller();
//    } else {
//        throw std::invalid_argument("Error, must enter a valid base command.");
//    }
}

controller::ControllerFactory::~ControllerFactory() = default;
