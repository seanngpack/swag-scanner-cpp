#include "IControllerGUI.h"
#include "SwagGUI.h"

std::string controller::IControllerGUI::get_name() {
    return gui->update_name();
}

int controller::IControllerGUI::get_deg() {
    return gui->update_deg();
}

int controller::IControllerGUI::get_rot() {
    return gui->update_rot();
}

void controller::IControllerGUI::update_console(const std::string &info) {
    gui->update_console(info);
}
