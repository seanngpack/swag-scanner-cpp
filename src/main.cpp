#include <iostream>
#include <Model.h>
#include <SR305.h>
#include "ScanController.h"
#include "ProcessingController.h"
#include "Visualizer.h"
#include "CloudType.h"
#include "ScanFileHandler.h"
#include "CalibrationFileHandler.h"


int main() {
    auto *camera = new camera::SR305();
    auto *arduino = new arduino::Arduino();
    std::shared_ptr<model::Model> model = std::make_shared<model::Model>();
    auto *viewer = new visual::Visualizer();
    std::shared_ptr<file::ScanFileHandler> file_handler = std::make_shared<file::ScanFileHandler>("test again");
//    std::shared_ptr<file::CalibrationFileHandler> file_handler = std::make_shared<file::CalibrationFileHandler>();


//
//    auto *scanController = new controller::ScanController(camera,
//                                                          arduino,
//                                                          model,
//                                                          file_handler);
    auto *processController = new controller::ProcessingController(model,
                                                                   viewer,
                                                                   file_handler);
//    scanController->scan(30, 12);
    processController->filter_clouds(CloudType::Type::RAW, .0003);
    processController->segment_clouds(CloudType::Type::FILTERED);
    processController->rotate_all_clouds(CloudType::Type::FILTERED);



//    delete scanController;
//    delete processController;
    delete viewer;
    return 0;
}
