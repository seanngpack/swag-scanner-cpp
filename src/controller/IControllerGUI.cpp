#include "IControllerGUI.h"
#include "SwagGUI.h"
#include <iostream>

controller::IControllerGUI::IControllerGUI(std::shared_ptr<SwagGUI> gui) :
        gui(std::move(gui)) {
    qRegisterMetaType<std::string>();
}
