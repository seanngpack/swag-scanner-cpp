#include "ProcessingControllerGUI.h"
#include "ProcessingController.h"
#include "ProcessingModel.h"
#include "FormsPayload.h"
#include "SwagGUI.h"


controller::ProcessingControllerGUI::ProcessingControllerGUI(std::shared_ptr<model::ProcessingModel> model,
                                                             std::shared_ptr<SwagGUI> gui) :
        ProcessingController(std::move(model)),
        IControllerGUI(std::move(gui)) {}

void controller::ProcessingControllerGUI::run() {
    emit update_console("Starting processing");
    ProcessingController::run();

    gui->display_cloud(model->get_cloud("REGISTERED.pcd"));
    emit update_console("Processing done!");
}

void controller::ProcessingControllerGUI::update(const IFormsPayload &payload) {
    const auto &p = dynamic_cast<const FormsPayload &>(payload);
    model->clear_clouds();
    model->set_scan(p.name);
}

