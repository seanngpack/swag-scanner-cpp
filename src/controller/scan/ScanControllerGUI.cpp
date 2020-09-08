#include "ScanControllerGUI.h"
#include "FormsPayload.h"
#include "ScanFileHandler.h"
#include <sys/time.h>


controller::ScanControllerGUI::ScanControllerGUI(std::shared_ptr<camera::ICamera> camera,
                                                 std::shared_ptr<arduino::Arduino> arduino,
                                                 std::shared_ptr<model::Model> model,
                                                 std::shared_ptr<file::ScanFileHandler> file_handler,
                                                 std::shared_ptr<SwagGUI> gui) :
        ScanController(std::move(camera),
                       std::move(arduino),
                       std::move(model),
                       std::move(file_handler)),
        IControllerGUI(std::move(gui)) {}

void controller::ScanControllerGUI::run() {
    ScanController::run();
    // i should split up ScanController's run methods into a bunch of protected helpers.
    // Then i can interweave them with update_console()
}

void controller::ScanControllerGUI::update(const IFormsPayload &payload) {
    const auto &p = dynamic_cast<const FormsPayload &>(payload);
    file_handler->set_scan(p.name);
    set_deg(p.deg);
    set_num_rot(p.rot);
}


