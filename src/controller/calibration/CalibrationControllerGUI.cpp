#include "CalibrationControllerGUI.h"
#include "SwagGUI.h"

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
        gui(std::move(gui)) {}

int controller::CalibrationControllerGUI::get_deg() {
    return gui->get_deg();
}

//TOTO: at some point do some checking to make sure deg and rot is not 0, if they are then use default values