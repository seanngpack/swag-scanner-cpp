#ifndef SWAG_SCANNER_CALIBRATIONCONTROLLER_H
#define SWAG_SCANNER_CALIBRATIONCONTROLLER_H

#include "Model.h"
#include "Arduino.h"
#include "SR305.h"
#include "FileHandler.h"
#include "Visualizer.h"

namespace controller {

    /**
     * This controller handles calibration.
     */
    class CalibrationController {
    public:
        CalibrationController(camera::ICamera *camera,
                              arduino::Arduino *arduino,
                              std::shared_ptr<model::Model> model,
                              std::shared_ptr<file::FileHandler> file_handler,
                              visual::Visualizer *viewer);

        void run();

        ~CalibrationController();

    private:
        camera::ICamera *camera;
        arduino::Arduino *arduino;
        std::shared_ptr<model::Model> model;
        std::shared_ptr<file::FileHandler> file_handler;
        visual::Visualizer *viewer;

        /**
         * Scan and save calibration clouds.
         * @param deg angle between scans.
         * @param n number of scans.
         */
        void scan(int deg, int n);
    };
}

#endif //SWAG_SCANNER_CALIBRATIONCONTROLLER_H
