#include <iostream>
#include <Model.h>
#include <SR305.h>
#include "ScanController.h"
#include "ProcessingController.h"
#include "Visualizer.h"
#include "../include/types/CloudType.h"
#include "../include/file/FileHandler.h"


int main() {
//    auto *camera = new camera::SR305();
//    auto *arduino = new arduino::Arduino();
    std::shared_ptr<model::Model> model = std::make_shared<model::Model>();
    auto *viewer = new visual::Visualizer();
    std::shared_ptr<file::FileHandler> file_handler = std::make_shared<file::FileHandler>(true);

//    std::shared_ptr<file::FileHandler> file_handler = std::make_shared<file::FileHandler>(true);
//    file_handler->set_scan_folder_path("/Users/seanngpack/Programming Stuff/Projects/scanner_files/18");


//    auto *scanController = new controller::ScanController(camera,
//                                                          arduino,
//                                                          model,
//                                                          file_handler);
//    auto *processController = new controller::ProcessingController(model,
//                                                                   viewer,
//                                                                   file_handler);
//    scanController->scan(30);
//    processController->filter_clouds(CloudType::Type::RAW, .0003);
//    processController->segment_clouds(file_handler->get_scan_folder_path(), CloudType::Type::FILTERED);
//    processController->register_all_clouds("/Users/seanngpack/Programming Stuff/Projects/scanner_files/18",
//                                           CloudType::Type::SEGMENTED);
//    processController->rotate_all_clouds(CloudType::Type::FILTERED);



//    delete scanController;
//    delete processController;
    return 0;
}
