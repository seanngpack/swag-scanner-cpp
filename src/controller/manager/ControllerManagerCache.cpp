#include "ControllerManagerCache.h"
#include "SR305.h"
#include "Arduino.h"
#include "CalibrationModel.h"
#include "ScanModel.h"
#include "ProcessingModel.h"
#include "Visualizer.h"
#include "CalibrationController.h"
#include "ProcessingController.h"
#include "ProcessingControllerGUI.h"
#include "MoveMethod.h"
#include "ScanController.h"
#include "ScanControllerGUI.h"
#include "MoveController.h"
#include "MoveControllerGUI.h"
#include "EditingControllerGUI.h"
#include "CalibrationControllerGUI.h"
#include "HomeController.h"
#include "SwagGUI.h"
#include "ControllerManager.h"
#include <iostream>

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG

#include "spdlog/spdlog.h"

controller::ControllerManagerCache::ControllerManagerCache(controller::ControllerManager *factory) :
        factory(factory),
        arduino(std::make_shared<arduino::Arduino>()) {}

std::shared_ptr<camera::SR305> controller::ControllerManagerCache::get_camera() {
    if (camera == nullptr) {
        camera = std::make_shared<camera::SR305>();
        std::cout << "memory address of camera: " << camera.get() << std::endl;
        return camera;
    }
    return camera;
}

std::shared_ptr<arduino::Arduino> controller::ControllerManagerCache::get_arduino() {
    if (arduino == nullptr) {
        arduino = std::make_shared<arduino::Arduino>();
        return arduino;
    }
    return arduino;
}

std::shared_ptr<model::CalibrationModel> controller::ControllerManagerCache::get_calibration_model() {
    if (calibration_model == nullptr) {
        calibration_model = std::make_shared<model::CalibrationModel>();
        return calibration_model;
    }
    return calibration_model;
}

std::shared_ptr<model::ScanModel> controller::ControllerManagerCache::get_scan_model() {
    if (scan_model == nullptr) {
        scan_model = std::make_shared<model::ScanModel>();
        return scan_model;
    }
    return scan_model;
}

std::shared_ptr<model::ProcessingModel> controller::ControllerManagerCache::get_processing_model() {
    if (processing_model == nullptr) {
        processing_model = std::make_shared<model::ProcessingModel>();
        return processing_model;
    }
    return processing_model;
}

std::shared_ptr<SwagGUI> controller::ControllerManagerCache::get_gui() {
    if (gui == nullptr) {
        gui = std::make_shared<SwagGUI>(nullptr, factory);
        gui->show();
        return gui;
    }
    return gui;
}

std::shared_ptr<controller::ScanController>
controller::ControllerManagerCache::get_scan_controller(const boost::program_options::variables_map &vm) {
    if (scan_controller != nullptr) {
        return scan_controller;
    }
    auto model = get_scan_model();
    if (vm.count("name")) {
        std::string name = vm["name"].as<std::string>();
        model->set_scan(name);
    }

    std::shared_ptr<controller::ScanController> controller = std::make_unique<controller::ScanController>(
            get_camera(),
            get_arduino(),
            model);

    if (vm.count("deg")) {
        controller->set_deg(vm["deg"].as<int>());
    }
    if (vm.count("rot")) {
        controller->set_num_rot(vm["rot"].as<int>());
    }
    scan_controller = controller;
    return controller;
}

std::shared_ptr<controller::ScanController> controller::ControllerManagerCache::get_scan_controller() {
    if (scan_controller == nullptr) {
        scan_controller = std::make_shared<controller::ScanController>(get_camera(),
                                                                       get_arduino(),
                                                                       get_scan_model());
        return scan_controller;
    }
    return scan_controller;
}

std::shared_ptr<controller::ScanControllerGUI> controller::ControllerManagerCache::get_scan_controller_gui() {
    if (scan_controller_gui == nullptr) {
        scan_controller_gui = std::make_shared<controller::ScanControllerGUI>(get_camera(),
                                                                              get_arduino(),
                                                                              get_scan_model(),
                                                                              get_gui());
        return scan_controller_gui;
    }
    return scan_controller_gui;
}

std::shared_ptr<controller::CalibrationController>
controller::ControllerManagerCache::get_calibration_controller(const boost::program_options::variables_map &vm) {
    if (calibration_controller != nullptr) {
        return calibration_controller;
    }
    auto model = get_calibration_model();
    if (vm.count("name")) {
        std::string c = vm["name"].as<std::string>().c_str();
        model->set_calibration(c);
    }

    std::shared_ptr<controller::CalibrationController> controller = std::make_unique<controller::CalibrationController>(
            get_camera(),
            get_arduino(),
            model);

    if (vm.count("deg")) {
        controller->set_deg(vm["deg"].as<int>());
    }
    if (vm.count("rot")) {
        controller->set_num_rot(vm["rot"].as<int>());
    }
    calibration_controller = controller;
    return controller;
}


std::shared_ptr<controller::CalibrationControllerGUI>
controller::ControllerManagerCache::get_calibration_controller_gui() {
    if (calibration_controller_gui == nullptr) {
        calibration_controller_gui = std::make_shared<controller::CalibrationControllerGUI>(get_camera(),
                                                                                            get_arduino(),
                                                                                            get_calibration_model(),
                                                                                            get_gui());
//        calibration_controller_gui->setup_gui();
        return calibration_controller_gui;
    }
    return calibration_controller_gui;
}

std::shared_ptr<controller::CalibrationController> controller::ControllerManagerCache::get_calibration_controller() {
    if (calibration_controller == nullptr) {
        calibration_controller = std::make_shared<controller::CalibrationController>(get_camera(),
                                                                                     get_arduino(),
                                                                                     get_calibration_model());
        return calibration_controller;
    }
    return calibration_controller;
}

std::shared_ptr<controller::ProcessingController>
controller::ControllerManagerCache::get_process_controller(const boost::program_options::variables_map &vm) {
    if (process_controller != nullptr) {
        return process_controller;
    }
    auto model = get_processing_model();
    if (vm.count("name")) {
        std::string c = vm["name"].as<std::string>().c_str();
        model->set_scan(c);
    }
    process_controller = std::make_unique<controller::ProcessingController>(model);
    return process_controller;
}

std::shared_ptr<controller::ProcessingController> controller::ControllerManagerCache::get_process_controller() {
    if (process_controller == nullptr) {
        process_controller = std::make_shared<controller::ProcessingController>(get_processing_model());
        return process_controller;
    }
    return process_controller;
}

std::shared_ptr<controller::ProcessingControllerGUI> controller::ControllerManagerCache::get_process_controller_gui() {
    if (process_controller_gui == nullptr) {
        process_controller_gui = std::make_shared<controller::ProcessingControllerGUI>(get_processing_model(),
                                                                                       get_gui());
        return process_controller_gui;
    }
    return process_controller_gui;
}

std::shared_ptr<controller::EditingControllerGUI> controller::ControllerManagerCache::get_edit_controller_gui() {
    if (edit_controller_gui == nullptr) {
        edit_controller_gui = std::make_shared<controller::EditingControllerGUI>(get_processing_model(),
                                                                                 get_gui());
        return edit_controller_gui;
    }
    return edit_controller_gui;
}

std::shared_ptr<controller::MoveController>
controller::ControllerManagerCache::get_move_controller(const boost::program_options::variables_map &vm) {
    std::shared_ptr<controller::MoveController> mc = std::make_unique<controller::MoveController>(
            get_arduino());
    if (vm.count("to")) {
        mc->set_deg(vm["to"].as<int>());
        mc->set_move_method(MoveMethod::TO);
    } else if (vm.count("by")) {
        mc->set_deg(vm["by"].as<int>());
        mc->set_move_method(MoveMethod::BY);
    } else if (vm.count("home")) {
        mc->set_deg(0);
        mc->set_move_method(MoveMethod::TO);
    }
    move_controller = mc;
    return mc;
}

std::shared_ptr<controller::MoveControllerGUI>
controller::ControllerManagerCache::get_move_controller_gui() {
    if (move_controller_gui == nullptr) {
        move_controller_gui = std::make_shared<controller::MoveControllerGUI>(get_arduino(),
                                                                              get_gui());
        return move_controller_gui;
    }
    return move_controller_gui;
}

std::shared_ptr<controller::HomeController>
controller::ControllerManagerCache::get_home_controller(const boost::program_options::variables_map &vm) {
    return std::make_shared<controller::HomeController>();
}


controller::ControllerManagerCache::~ControllerManagerCache() {
    spdlog::get("swag_logger")->debug("ControllerManagerCache destructor called");
}

