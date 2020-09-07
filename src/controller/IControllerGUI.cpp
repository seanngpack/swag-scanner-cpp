#include "IControllerGUI.h"
#include "SwagGUI.h"
#include "IFileHandler.h"
#include <iostream>


std::vector<std::string> controller::IControllerGUI::get_all_scans() {
    return file::IFileHandler::get_all_scans();
}

std::vector<std::string> controller::IControllerGUI::get_all_calibrations() {
    return file::IFileHandler::get_all_calibrations();
}
