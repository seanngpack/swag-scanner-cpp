#ifndef SWAG_SCANNER_CLICLIENT_H
#define SWAG_SCANNER_CLICLIENT_H

#include "Model.h"
#include "Visualizer.h"
#include "ScanFileHandler.h"
#include "CalibrationFileHandler.h"

namespace cli {
    /**
     * This class keeps track of shared resources and uses the manager to create
     * new controllers;
     */
    class CLIClient {
    public:
        controller::IController get_controller();


    private:
        std::shared_ptr<model::Model> model;
        std::shared_ptr<visual::Visualizer> viewer;
        std::shared_ptr<file::ScanFileHandler> scan_file_handler;
        std::shared_ptr<file::CalibrationFileHandler> calibration_file_handler;
    };

};

#endif //SWAG_SCANNER_CLICLIENT_H
