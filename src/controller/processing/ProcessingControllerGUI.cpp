#include "ProcessingControllerGUI.h"
#include "ProcessingController.h"
#include "ScanFileHandler.h"
#include "FormsPayload.h"


controller::ProcessingControllerGUI::ProcessingControllerGUI(std::unique_ptr<model::ProcessingModel> model,
                                                             std::shared_ptr<SwagGUI> gui) :
        ProcessingController(std::move(model)),
        IControllerGUI(std::move(gui)) {}

void controller::ProcessingControllerGUI::run() {
    emit update_console("Starting processing");
    ProcessingController::run();
    emit update_console("Processing done!");
}

void controller::ProcessingControllerGUI::update(const IFormsPayload &payload) {
    const auto &p = dynamic_cast<const FormsPayload &>(payload);
    if (p.name.empty()) {
        return;
    }
    file_handler->set_scan(p.name);
}

