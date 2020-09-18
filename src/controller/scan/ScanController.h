#ifndef SWAG_SCANNER_SCANCONTROLLER_H
#define SWAG_SCANNER_SCANCONTROLLER_H

#include "IController.h"
#include <memory>

namespace visual {
    class Visualizer;
}

namespace model {
    class ScanModel;
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
    class ScanController : public virtual IController {
    public:
        ScanController(std::shared_ptr<camera::ICamera> camera,
                       std::shared_ptr<arduino::Arduino> arduino,
                       std::shared_ptr<model::ScanModel> model);

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
        std::shared_ptr<model::ScanModel> model;
        int deg = 20;
        int num_rot = 18;

    };
}

#endif //SWAG_SCANNER_SCANCONTROLLER_H
