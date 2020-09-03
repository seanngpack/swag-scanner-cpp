#ifndef SWAG_SCANNER_FILTERTESTINGCONTROLLER_H
#define SWAG_SCANNER_FILTERTESTINGCONTROLLER_H

#include "IController.h"
#include <memory>

namespace model {
    class Model;
}

namespace arduino {
    class Arduino;
}

namespace file {
    class ScanFileHandler;
}

namespace visual {
    class Visualizer;
}

namespace camera {
    class SR305;
}

namespace controller {
    /**
     * This controller is used for testing depth frame filtering. Can grab single depth frames
     * and display their point clouds with or without filtering.
     */
    class FilterTestingController : public IController {
    public:
        FilterTestingController(std::shared_ptr<camera::SR305> camera,
                                std::shared_ptr<model::Model> model,
                                std::shared_ptr<file::ScanFileHandler> file_handler,
                                std::shared_ptr<visual::Visualizer> viewer);

        /**
         * Compare raw depth data to filtered depth data in a side by side visualization.
         */
        void run() override;

    private:
        std::shared_ptr<camera::SR305> camera;
        std::shared_ptr<model::Model> model;
        std::shared_ptr<file::ScanFileHandler> file_handler;
        std::shared_ptr<visual::Visualizer> viewer;
    };
}

#endif //SWAG_SCANNER_FILTERTESTINGCONTROLLER_H
