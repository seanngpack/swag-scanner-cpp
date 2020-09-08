#ifndef SWAG_SCANNER_SCANCONTROLLER_H
#define SWAG_SCANNER_SCANCONTROLLER_H

#include "IController.h"
#include <memory>

namespace visual {
    class Visualizer;
}

namespace model {
    class Model;
}

namespace file {
    class ScanFileHandler;
}

namespace arduino {
    class Arduino;
}

namespace camera {
    class ICamera;
}

namespace controller {
    /**
    * This controller handles data acquisition.
    */
    class ScanController : public IController {
    public:
        ScanController(std::shared_ptr<camera::ICamera> camera,
                       std::shared_ptr<arduino::Arduino> arduino,
                       std::shared_ptr<model::Model> model,
                       std::shared_ptr<file::ScanFileHandler> file_handler);

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


    protected:
        std::shared_ptr<camera::ICamera> camera;
        std::shared_ptr<arduino::Arduino> arduino;
        std::shared_ptr<model::Model> model;
        std::shared_ptr<file::ScanFileHandler> file_handler;
        int deg = 20;
        int num_rot = 18;

        void update_json_time();
    };
}

#endif //SWAG_SCANNER_SCANCONTROLLER_H
