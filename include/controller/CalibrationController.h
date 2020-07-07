#ifndef SWAG_SCANNER_CALIBRATIONCONTROLLER_H
#define SWAG_SCANNER_CALIBRATIONCONTROLLER_H

#include "Model.h"
#include "Arduino.h"
#include "SR305.h"
#include "FileHandler.h"
#include "Visualizer.h"
#include "ScanController.h"
#include "Point.h"

namespace controller {

    /**
     * This controller handles calibration.
     */
    class CalibrationController {
    public:
        CalibrationController(std::unique_ptr<controller::ScanController> scan_controller,
                              std::shared_ptr<model::Model> model,
                              std::shared_ptr<file::FileHandler> file_handler,
                              visual::Visualizer *viewer,
                              int deg,
                              int num_rot);

        /**
         * Scan calibration fixture with member info for degs and # of rotations into a new
         * calibration folder. Calculate configuration properties and save to that folder.
         *
         */
        void run();

        ~CalibrationController();

    private:
        std::unique_ptr<controller::ScanController> scan_controller;
        std::shared_ptr<model::Model> model;
        std::shared_ptr<file::FileHandler> file_handler;
        visual::Visualizer *viewer;
        int deg;
        int num_rot;

    };
}

#endif //SWAG_SCANNER_CALIBRATIONCONTROLLER_H
