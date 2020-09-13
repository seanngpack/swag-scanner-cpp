#include "CalibrationControllerGUI.h"
#include "CalibrationFileHandler.h"
#include "Model.h"
#include "Normal.h"
#include "Point.h"
#include "SR305.h"
#include "Arduino.h"
#include "Visualizer.h"
#include "SwagGUI.h"
#include "FormsPayload.h"

controller::CalibrationControllerGUI::CalibrationControllerGUI(std::shared_ptr<camera::ICamera> camera,
                                                               std::shared_ptr<arduino::Arduino> arduino,
                                                               std::shared_ptr<model::Model> model,
                                                               std::shared_ptr<file::CalibrationFileHandler> file_handler,
                                                               std::shared_ptr<visual::Visualizer> viewer,
                                                               std::shared_ptr<SwagGUI> gui) :
        CalibrationController(std::move(camera),
                              std::move(arduino),
                              std::move(model),
                              std::move(file_handler),
                              std::move(viewer)),
        IControllerGUI(std::move(gui)) {}


void controller::CalibrationControllerGUI::run() {
    emit update_console("Starting scan");
    scan();
    emit update_console("Scan complete");
    emit update_console("Calculating calibration planes...");
    get_calibration_planes();
    emit update_console("Calculation complete");
    emit update_console("Calculating center point and rotation axis...");
    calculate();
    emit update_console("Calculation complete");
}


void controller::CalibrationControllerGUI::update(const IFormsPayload &payload) {
    const auto &p = dynamic_cast<const FormsPayload &>(payload);
    file_handler->set_calibration(p.name);
    this->deg = p.deg;
    this->num_rot = p.rot;
}
