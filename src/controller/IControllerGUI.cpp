#include "IControllerGUI.h"
#include "SwagGUI.h"
#include "IFileHandler.h"
#include <iostream>

void controller::IControllerGUI::update_name() {
    // do nothing
//    return gui->update_name();
}

void controller::IControllerGUI::update_deg() {
//    return gui->update_deg();
}

void controller::IControllerGUI::update_rot() {
//    return gui->update_rot();
}

void controller::IControllerGUI::update_console(const std::string &info) {
//    gui->update_console(info);
}

std::vector<std::string> controller::IControllerGUI::get_all_scans() {
    return file::IFileHandler::get_all_scans();
}

std::vector<std::string> controller::IControllerGUI::get_all_calibrations() {
    return file::IFileHandler::get_all_calibrations();
}
