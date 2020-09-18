#ifndef SWAG_SCANNER_PROCESSINGCONTROLLER_H
#define SWAG_SCANNER_PROCESSINGCONTROLLER_H

#include "IController.h"
#include "CloudType.h"
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace visual {
    class Visualizer;
}

namespace model {
    class ProcessingModel;
}

namespace file {
    class ScanFileHandler;
}


namespace controller {

    /**
     * This controller handles data processing commands.
     */
    class ProcessingController : public virtual IController {
    public:
        explicit ProcessingController(std::shared_ptr<model::ProcessingModel> model);

        /**
        * Process the data. Filters, segments, and rotates the clouds.
        */
        void run() override;


    protected:
        std::shared_ptr<model::ProcessingModel> model;
        std::shared_ptr<visual::Visualizer> viewer;
        std::shared_ptr<file::ScanFileHandler> file_handler;
    };


}

#endif //SWAG_SCANNER_PROCESSINGCONTROLLER_H
