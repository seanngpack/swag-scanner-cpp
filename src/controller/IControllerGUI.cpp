#include "IControllerGUI.h"
#include "SwagGUI.h"
#include "IFileHandler.h"
#include <iostream>

controller::IControllerGUI::IControllerGUI(std::shared_ptr<SwagGUI> gui) :
        gui(std::move(gui)) {
    qRegisterMetaType<std::string>();
}

std::vector<std::string> controller::IControllerGUI::get_all_scans() {
    return file::IFileHandler::get_all_scans();
}

std::vector<std::string> controller::IControllerGUI::get_all_calibrations() {
    return file::IFileHandler::get_all_calibrations();
}
