#ifndef SWAG_SCANNER_CALIBRATIONCONTROLLERGUI_H
#define SWAG_SCANNER_CALIBRATIONCONTROLLERGUI_H

#include "CalibrationController.h"

class SwagGUI;

namespace controller {
    class CalibrationControllerGUI : public CalibrationController {
    public:
        CalibrationControllerGUI(std::shared_ptr<camera::ICamera> camera,
                                 std::shared_ptr<arduino::Arduino> arduino,
                                 std::shared_ptr<model::Model> model,
                                 std::shared_ptr<file::CalibrationFileHandler> file_handler,
                                 std::shared_ptr<visual::Visualizer> viewer,
                                 std::shared_ptr<SwagGUI> gui);


    private:
        std::shared_ptr<SwagGUI> gui;

        /**
         * Get the degree from GUI.
         * @return degree.
         */
        int get_deg();
    };

}

#endif //SWAG_SCANNER_CALIBRATIONCONTROLLERGUI_H
