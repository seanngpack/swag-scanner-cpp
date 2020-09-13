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
    class Model;
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
                              std::shared_ptr<model::Model> model,
                              std::shared_ptr<file::CalibrationFileHandler> file_handler,
                              std::shared_ptr<visual::Visualizer> viewer);

        /**
         * Scan calibration fixture with member info for degs and # of rotations into a new
         * calibration folder. Calculate configuration properties and save to that folder.
         */
        void run() override;

        void set_deg(int deg);

        void set_num_rot(int num_rot);


    protected:
        std::shared_ptr<camera::ICamera> camera;
        std::shared_ptr<arduino::Arduino> arduino;
        std::shared_ptr<model::Model> model;
        std::shared_ptr<file::CalibrationFileHandler> file_handler;
        std::shared_ptr<visual::Visualizer> viewer;
        int deg = 15;
        int num_rot = 8;
        std::vector<equations::Plane> upright_planes;
        std::vector<equations::Plane> ground_planes;
        std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> clouds;


        /**
         * Scan the calibration clouds and save them into current folder.
         */
        void scan();

        /**
         * Load the recently scanned calibration clouds and calculate the planes equations. Store them
         * into class members.
         */
        void get_calibration_planes();

        /**
         * Perform center point calculations and then refine the calculation.
         * Also update JSON file with the center point coordinate and axis of rotation direction.
         */
        void calculate();

    };
}

#endif //SWAG_SCANNER_CALIBRATIONCONTROLLER_H
