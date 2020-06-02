#ifndef SWAG_SCANNER_CONTROLLER_H
#define SWAG_SCANNER_CONTROLLER_H

#include "Model.h"
#include "Arduino.h"
#include "SR305.h"
#include "Visualizer.h"

namespace controller {
    class Controller {
    public:
        Controller(camera::ICamera *camera,
                   arduino::Arduino *arduino,
                   model::Model *model,
                   visual::Visualizer *viewer);


        /**
         * Write folders, run the scan and collect data.
         * @param degs number of degrees per rotation interval.
         */
        void scan(int degs);

        /**
         * Process the data. Currently will process the most recently scanned.
         */
        void process_data();

        void visualize_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);


        ~Controller();

    private:
        camera::ICamera *camera;
        arduino::Arduino *arduino;
        model::Model *model;
        visual::Visualizer *viewer;
    };
}

#endif //SWAG_SCANNER_CONTROLLER_H
