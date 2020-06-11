#include <iostream>
#include <Model.h>
#include <SR305.h>
#include "ScanController.h"
#include "ProcessingController.h"
#include "Visualizer.h"
#include "CloudType.h"
#include "FileHandler.h"


int main() {
    camera::ICamera *camera = new camera::SR305();
    arduino::Arduino *arduino = new arduino::Arduino();
    std::shared_ptr<model::Model> model = std::make_shared<model::Model>();
    visual::Visualizer *viewer = new visual::Visualizer();
    std::shared_ptr<file::FileHandler> file_handler = std::make_shared<file::FileHandler>(true);


    controller::ScanController *scanController = new controller::ScanController(camera,
                                                                                arduino,
                                                                                model,
                                                                                file_handler);
    controller::ProcessingController *processController = new controller::ProcessingController(model,
                                                                                               viewer,
                                                                                               file_handler);
//    controller->scan(9);

    processController->register_all_clouds("/Users/seanngpack/Programming Stuff/Projects/scanner_files/17",
                                    CloudType::Type::RAW);

    delete processController;
    return 0;
}
