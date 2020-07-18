#ifndef SWAG_SCANNER_CALIBRATIONCONTROLLER_H
#define SWAG_SCANNER_CALIBRATIONCONTROLLER_H

#include "IController.h"
#include "Model.h"
#include "Arduino.h"
#include "SR305.h"
#include "Visualizer.h"
#include "CalibrationFileHandler.h"
#include "ScanController.h"
#include "Point.h"

namespace controller {

    /**
     * This controller handles calibration.
     */
    class CalibrationController : public IController {
    public:
        CalibrationController(std::shared_ptr<camera::ICamera> camera,
                              std::shared_ptr<arduino::Arduino> arduino,
                              std::shared_ptr<model::Model> model,
                              std::shared_ptr<file::CalibrationFileHandler> file_handler,
                              std::shared_ptr<visual::Visualizer> viewer);

        /**
         * Scan calibration fixture with member info for degs and # of rotations into a new
         * calibration folder. Calculate configuration properties and save to that folder.
         *
         */
        void run() override;

        void set_deg(int deg);

        void set_num_rot(int num_rot);


    private:
        std::shared_ptr<camera::ICamera> camera;
        std::shared_ptr<arduino::Arduino> arduino;
        std::shared_ptr<model::Model> model;
        std::shared_ptr<file::CalibrationFileHandler> file_handler;
        std::shared_ptr<visual::Visualizer> viewer;
        int deg = 15;
        int num_rot = 7;

        /**
         * Scan the calibration clouds and save them into current folder.
         */
        void scan();

    };
}

#endif //SWAG_SCANNER_CALIBRATIONCONTROLLER_H
