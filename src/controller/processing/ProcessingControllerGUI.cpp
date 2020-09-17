#include "ProcessingController.h"
#include "ProcessingControllerGUI.h"
#include "ScanFileHandler.h"
#include "FormsPayload.h"


controller::ProcessingControllerGUI::ProcessingControllerGUI(std::shared_ptr<model::Model> model,
                                                             std::shared_ptr<visual::Visualizer> viewer,
                                                             std::shared_ptr<file::ScanFileHandler> file_handler,
                                                             std::shared_ptr<SwagGUI> gui) :
        ProcessingController(std::move(model),
                             std::move(viewer),
                             std::move(file_handler)),
        IControllerGUI(std::move(gui)) {}

void controller::ProcessingControllerGUI::run() {
    emit update_console("Starting processing");
    ProcessingController::run();
    emit update_console("Processing done!");
}

void controller::ProcessingControllerGUI::update(const IFormsPayload &payload) {
    const auto &p = dynamic_cast<const FormsPayload &>(payload);
    file_handler->set_scan(p.name);
}

