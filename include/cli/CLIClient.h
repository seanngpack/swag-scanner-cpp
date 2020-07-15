#ifndef SWAG_SCANNER_CLICLIENT_H
#define SWAG_SCANNER_CLICLIENT_H

#include "Model.h"
#include "Visualizer.h"
#include "SR305.h"
#include "Arduino.h"
#include "ScanFileHandler.h"
#include "CalibrationFileHandler.h"
#include "IController.h"

namespace cli {
    /**
     * This class keeps track of shared resources and uses the manager to create
     * new controllers;
     */
    class CLIClient {
    public:

        CLIClient();

        std::unique_ptr<controller::IController> get_controller(int argc, char* argv[]);


    private:
//        std::shared_ptr<camera::SR305> camera;
//        std::shared_ptr<arduino::Arduino> arduino;
//        std::shared_ptr<model::Model> model;
//        std::shared_ptr<visual::Visualizer> viewer;
//        std::shared_ptr<file::ScanFileHandler> scan_file_handler;
//        std::shared_ptr<file::CalibrationFileHandler> calibration_file_handler;
    };

};

#endif //SWAG_SCANNER_CLICLIENT_H
