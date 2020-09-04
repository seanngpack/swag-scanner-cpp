#include "ControllerFactoryCache.h"

std::shared_ptr<camera::SR305> controller::ControllerFactoryCache::get_camera() {
    if (camera == nullptr) {
        return std::make_shared<camera::SR305>();
    }
    return camera;
}

std::shared_ptr<arduino::Arduino> controller::ControllerFactoryCache::get_arduino() {
    if (arduino == nullptr) {
        return std::make_shared<arduino::Arduino>();
    }
    return arduino;
}

std::shared_ptr<model::Model> controller::ControllerFactoryCache::get_model() {
    if (model == nullptr) {
        return std::make_shared<model::Model>();
    }
    return model;
}

std::shared_ptr<visual::Visualizer> controller::ControllerFactoryCache::get_viewer() {
    if (viewer == nullptr) {
        return std::make_shared<visual::Visualizer>();
    }
    return viewer;
}

std::shared_ptr<file::ScanFileHandler> controller::ControllerFactoryCache::get_scan_file_handler() {
    if (scan_file_handler == nullptr) {
        return std::make_shared<file::ScanFileHandler>();
    }
    return scan_file_handler;
}

std::shared_ptr<file::CalibrationFileHandler> controller::ControllerFactoryCache::get_calibration_file_handler() {
    if (calibration_file_handler == nullptr) {
        return std::make_shared<file::CalibrationFileHandler>();
    }
    return calibration_file_handler;
}
