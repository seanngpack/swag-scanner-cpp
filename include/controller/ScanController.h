#ifndef SWAG_SCANNER_SCANCONTROLLER_H
#define SWAG_SCANNER_SCANCONTROLLER_H

#include "IController.h"
#include "Model.h"
#include "Arduino.h"
#include "SR305.h"
#include "Visualizer.h"
#include "ScanFileHandler.h"


namespace controller {
    /**
    * This controller handles data acquisition.
    */
    class ScanController : public IController {
    public:
        ScanController(std::shared_ptr<camera::ICamera> camera,
                       std::shared_ptr<arduino::Arduino> arduino,
                       std::shared_ptr<model::Model> model,
                       std::shared_ptr<file::ScanFileHandler> file_handler,
                       int deg = 20,
                       int num_rot = 18);

        void run() override;

        void set_deg(int deg);

        void set_num_rot(int num_rot);

        /**
         * Write folders, run the scan and collect data.
         * @param degs number of degrees per rotation.
         * @param num_rot number of rotations.
         * the scan to (RAW, CALIBRATION, etc)
         */
        void scan();


    private:
        std::shared_ptr<camera::ICamera> camera;
        std::shared_ptr<arduino::Arduino> arduino;
        std::shared_ptr<model::Model> model;
        std::shared_ptr<file::ScanFileHandler> file_handler;
        int deg;
        int num_rot;
    };
}

#endif //SWAG_SCANNER_SCANCONTROLLER_H
