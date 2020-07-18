#ifndef SWAG_SCANNER_FILTERTESTINGCONTROLLER_H
#define SWAG_SCANNER_FILTERTESTINGCONTROLLER_H

#include "IController.h"
#include "Model.h"
#include "Arduino.h"
#include "SR305.h"
#include "Visualizer.h"
#include "ScanFileHandler.h"

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

        void run() override;

    private:
        std::shared_ptr<camera::SR305> camera;
        std::shared_ptr<model::Model> model;
        std::shared_ptr<file::ScanFileHandler> file_handler;
        std::shared_ptr<visual::Visualizer> viewer;
    };
}

#endif //SWAG_SCANNER_FILTERTESTINGCONTROLLER_H
