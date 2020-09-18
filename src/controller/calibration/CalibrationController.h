#ifndef SWAG_SCANNER_CALIBRATIONCONTROLLER_H
#define SWAG_SCANNER_CALIBRATIONCONTROLLER_H

#include "IController.h"
#include "Plane.h"
#include <vector>

namespace controller {
    class ScanController;
}

namespace file {
    class CalibrationFileHandler;
}

namespace arduino {
    class Arduino;
}

namespace model {
    class CalibrationModel;
}

namespace visual {
    class Visualizer;
}

namespace camera {
    class ICamera;
}

namespace pcl {
    class PointXYZ;

    template<class pointT>
    class PointCloud;
}


namespace controller {
    /**
     * This controller handles calibration.
     */
    class CalibrationController : public virtual IController {
    public:
        CalibrationController(std::shared_ptr<camera::ICamera> camera,
                              std::shared_ptr<arduino::Arduino> arduino,
                              std::shared_ptr<model::CalibrationModel> model);

        /**
         * Scan calibration fixture with member info for degs and # of rotations into a new
         * calibration folder. Calculate configuration properties and save to that folder.
         */
        void run() override;

        void set_deg(int deg);

        void set_num_rot(int rot);


    protected:
        std::shared_ptr<camera::ICamera> camera;
        std::shared_ptr<arduino::Arduino> arduino;
        std::shared_ptr<model::CalibrationModel> model;
        int deg = 15;
        int num_rot = 8;
        std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> clouds;


        /**
         * Scan the calibration clouds and save them into current folder.
         */
        void scan();


    };
}

#endif //SWAG_SCANNER_CALIBRATIONCONTROLLER_H
