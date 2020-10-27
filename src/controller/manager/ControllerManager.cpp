#include "ControllerManager.h"
#include "IController.h"
#include "IControllerGUI.h"
#include "SR305.h"
#include "Arduino.h"
#include "CalibrationModel.h"
#include "ScanModel.h"
#include "ProcessingModel.h"
#include "Visualizer.h"
#include "CalibrationFileHandler.h"
#include "CalibrationController.h"
#include "ProcessingController.h"
#include "ProcessingControllerGUI.h"
#include "ScanController.h"
#include "ScanControllerGUI.h"
#include "MoveController.h"
#include "MoveControllerGUI.h"
#include "CalibrationControllerGUI.h"
#include "EditingControllerGUI.h"
#include "HomeController.h"
#include "ControllerManagerCache.h"
#include "MoveController.h"
#include "SwagGUI.h"
#include "Logger.h"


namespace po = boost::program_options;

controller::ControllerManager::ControllerManager() : cache(std::make_unique<ControllerManagerCache>(this)) {}

controller::ControllerManager::~ControllerManager() {
  logger::debug("ControllerManger ~destructor called");
}

std::shared_ptr<controller::IController> controller::ControllerManager::get_controller(const po::variables_map &vm) {
    if (vm.count("scan")) {
        return cache->get_scan_controller(vm);
    } else if (vm.count("calibrate")) {
        return cache->get_calibration_controller(vm);
    } else if (vm.count("process")) {
        return cache->get_process_controller(vm);
    } else if (vm.count("move")) {
        return cache->get_move_controller(vm);
    } else if (vm.count("set_home")) {
        return cache->get_home_controller(vm);
    } else {
        throw std::invalid_argument("Error, must enter a valid base command.");
    }
}

std::shared_ptr<controller::IController> controller::ControllerManager::get_controller(const std::string &name) {
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

std::shared_ptr<controller::IControllerGUI> controller::ControllerManager::get_gui_controller(const std::string &name) {
    if (name == "scan") {
        return cache->get_scan_controller_gui();
    } else if (name == "calibrate") {
        return cache->get_calibration_controller_gui();
    } else if (name == "move") {
        return cache->get_move_controller_gui();
    } else if (name == "process") {
        return cache->get_process_controller_gui();
    } else if (name == "edit") {
        return cache->get_edit_controller_gui();
    } else {
        throw std::invalid_argument("Error, must enter a valid controller name.");
    }
}

std::shared_ptr<SwagGUI> controller::ControllerManager::get_gui() {
    return cache->get_gui();
}
