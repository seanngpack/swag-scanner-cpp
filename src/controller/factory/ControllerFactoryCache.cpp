#include "ControllerFactoryCache.h"
#include "SR305.h"
#include "Arduino.h"
#include "Model.h"
#include "Visualizer.h"
#include "ScanFileHandler.h"
#include "CalibrationFileHandler.h"
#include "CalibrationController.h"
#include "ProcessingController.h"
#include "ProcessingControllerGUI.h"
#include "MoveMethod.h"
#include "ScanController.h"
#include "ScanControllerGUI.h"
#include "MoveController.h"
#include "MoveControllerGUI.h"
#include "FilterTestingController.h"
#include "CalibrationControllerGUI.h"
#include "HomeController.h"
#include "SwagGUI.h"
#include "ControllerFactory.h"
#include <iostream>

controller::ControllerFactoryCache::ControllerFactoryCache(controller::ControllerFactory *factory) :
        factory(factory),
        model(std::make_shared<model::Model>()),
        scan_file_handler(std::make_shared<file::ScanFileHandler>()),
        calibration_file_handler(std::make_shared<file::CalibrationFileHandler>()) {}

std::shared_ptr<camera::SR305> controller::ControllerFactoryCache::get_camera() {
    if (camera == nullptr) {
        camera = std::make_shared<camera::SR305>();
        std::cout << "memory address of camera: " << camera.get() << std::endl;
        return camera;
    }
    return camera;
}

std::shared_ptr<arduino::Arduino> controller::ControllerFactoryCache::get_arduino() {
    if (arduino == nullptr) {
        arduino = std::make_shared<arduino::Arduino>();
        return arduino;
    }
    return arduino;
}

std::shared_ptr<model::Model> controller::ControllerFactoryCache::get_model() {
    if (model == nullptr) {
        model = std::make_shared<model::Model>();
        return model;
    }
    return model;
}

std::shared_ptr<visual::Visualizer> controller::ControllerFactoryCache::get_viewer() {
    if (viewer == nullptr) {
        viewer = std::make_shared<visual::Visualizer>();
        return viewer;
    }
    return viewer;
}

std::shared_ptr<file::ScanFileHandler> controller::ControllerFactoryCache::get_scan_file_handler() {
    if (scan_file_handler == nullptr) {
        scan_file_handler = std::make_shared<file::ScanFileHandler>();
        return scan_file_handler;
    }
    return scan_file_handler;
}

std::shared_ptr<file::CalibrationFileHandler> controller::ControllerFactoryCache::get_calibration_file_handler() {
    if (calibration_file_handler == nullptr) {
        calibration_file_handler = std::make_shared<file::CalibrationFileHandler>();
        return calibration_file_handler;
    }
    return calibration_file_handler;
}

std::shared_ptr<SwagGUI> controller::ControllerFactoryCache::get_gui() {
    if (gui == nullptr) {
        gui = std::make_shared<SwagGUI>(factory);
        gui->show();
        return gui;
    }
    return gui;
}

std::shared_ptr<controller::ScanController>
controller::ControllerFactoryCache::get_scan_controller(const boost::program_options::variables_map &vm) {
    if (scan_controller != nullptr) {
        return scan_controller;
    }
    std::shared_ptr<file::ScanFileHandler> file_handler;
    if (vm.count("name")) {
        std::string name = vm["name"].as<std::string>();
        file_handler = get_scan_file_handler();
        file_handler->set_scan(name);
    } else {
        file_handler = get_scan_file_handler();
    }

    std::shared_ptr<controller::ScanController> controller = std::make_unique<controller::ScanController>(
            get_camera(),
            get_arduino(),
            get_model(),
            file_handler);

    if (vm.count("deg")) {
        controller->set_deg(vm["deg"].as<int>());
    }
    if (vm.count("rot")) {
        controller->set_num_rot(vm["rot"].as<int>());
    }
    scan_controller = controller;
    return controller;
}

std::shared_ptr<controller::ScanController> controller::ControllerFactoryCache::get_scan_controller() {
    if (scan_controller == nullptr) {
        scan_controller = std::make_shared<controller::ScanController>(get_camera(),
                                                                       get_arduino(),
                                                                       get_model(),
                                                                       get_scan_file_handler());
        return scan_controller;
    }
    return scan_controller;
}

std::shared_ptr<controller::ScanControllerGUI> controller::ControllerFactoryCache::get_scan_controller_gui() {
    if (scan_controller_gui == nullptr) {
        scan_controller_gui = std::make_shared<controller::ScanControllerGUI>(get_camera(),
                                                                              get_arduino(),
                                                                              get_model(),
                                                                              get_scan_file_handler(),
                                                                              get_gui());
        return scan_controller_gui;
    }
    return scan_controller_gui;
}

std::shared_ptr<controller::CalibrationController>
controller::ControllerFactoryCache::get_calibration_controller(const boost::program_options::variables_map &vm) {
    if (calibration_controller != nullptr) {
        return calibration_controller;
    }
    std::shared_ptr<file::CalibrationFileHandler> file_handler;
    if (vm.count("name")) {
        const char *c = vm["name"].as<std::string>().c_str();
        file_handler = std::make_shared<file::CalibrationFileHandler>(c);
    } else {
        file_handler = std::make_shared<file::CalibrationFileHandler>();
    }

    std::shared_ptr<controller::CalibrationController> controller = std::make_unique<controller::CalibrationController>(
            get_camera(),
            get_arduino(),
            get_model(),
            file_handler,
            get_viewer());

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
controller::ControllerFactoryCache::get_calibration_controller_gui() {
    if (calibration_controller_gui == nullptr) {
        calibration_controller_gui = std::make_shared<controller::CalibrationControllerGUI>(get_camera(),
                                                                                            get_arduino(),
                                                                                            get_model(),
                                                                                            get_calibration_file_handler(),
                                                                                            get_viewer(),
                                                                                            get_gui());
//        calibration_controller_gui->setup_gui();
        return calibration_controller_gui;
    }
    return calibration_controller_gui;
}

std::shared_ptr<controller::CalibrationController> controller::ControllerFactoryCache::get_calibration_controller() {
    if (calibration_controller == nullptr) {
        calibration_controller = std::make_shared<controller::CalibrationController>(get_camera(),
                                                                                     get_arduino(),
                                                                                     get_model(),
                                                                                     get_calibration_file_handler(),
                                                                                     get_viewer());
        return calibration_controller;
    }
    return calibration_controller;
}

std::shared_ptr<controller::ProcessingController>
controller::ControllerFactoryCache::get_process_controller(const boost::program_options::variables_map &vm) {
    if (process_controller != nullptr) {
        return process_controller;
    }
    std::shared_ptr<file::ScanFileHandler> file_handler;
    if (vm.count("name")) {
        const char *c = vm["name"].as<std::string>().c_str();
        file_handler = std::make_shared<file::ScanFileHandler>(c);
    } else {
        file_handler = std::make_shared<file::ScanFileHandler>();
    }
    process_controller = std::make_unique<controller::ProcessingController>(get_model(),
                                                                            get_viewer(),
                                                                            file_handler);
    return process_controller;
}

std::shared_ptr<controller::ProcessingController> controller::ControllerFactoryCache::get_process_controller() {
    if (process_controller == nullptr) {
        process_controller = std::make_shared<controller::ProcessingController>(get_model(),
                                                                                get_viewer(),
                                                                                get_scan_file_handler());
        return process_controller;
    }
    return process_controller;
}

std::shared_ptr<controller::ProcessingControllerGUI> controller::ControllerFactoryCache::get_process_controller_gui() {
    if (process_controller_gui == nullptr) {
        process_controller_gui = std::make_shared<controller::ProcessingControllerGUI>(get_model(),
                                                                                       get_viewer(),
                                                                                       get_scan_file_handler(),
                                                                                       get_gui());
        return process_controller_gui;
    }
    return process_controller_gui;
}

std::shared_ptr<controller::FilterTestingController>
controller::ControllerFactoryCache::get_filter_testing_controller(const boost::program_options::variables_map &vm) {
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
    return std::make_unique<controller::FilterTestingController>(get_camera(),
                                                                 get_model(),
                                                                 get_scan_file_handler(),
                                                                 std::make_shared<visual::Visualizer>());
}

std::shared_ptr<controller::MoveController>
controller::ControllerFactoryCache::get_move_controller(const boost::program_options::variables_map &vm) {
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
controller::ControllerFactoryCache::get_move_controller_gui() {
    if (move_controller_gui == nullptr) {
        move_controller_gui = std::make_shared<controller::MoveControllerGUI>(get_arduino(),
                                                                              get_gui());
        return move_controller_gui;
    }
    return move_controller_gui;
}

std::shared_ptr<controller::HomeController>
controller::ControllerFactoryCache::get_home_controller(const boost::program_options::variables_map &vm) {
    return std::make_shared<controller::HomeController>();
}


controller::ControllerFactoryCache::~ControllerFactoryCache() = default;

