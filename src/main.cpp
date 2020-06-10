#include <iostream>
#include <Model.h>
#include <SR305.h>
#include "Controller.h"
#include "Visualizer.h"
#include "CloudType.h"
#include "FileHandler.h"


int main() {
    camera::ICamera *camera = new camera::SR305();
    arduino::Arduino *arduino = new arduino::Arduino();
    model::Model *model = new model::Model();
    visual::Visualizer *viewer = new visual::Visualizer();
    file::FileHandler *file_handler = new file::FileHandler(true);


    controller::Controller *controller = new controller::Controller(camera,
            arduino, model, viewer, file_handler);
//    controller->scan(9);

    controller->register_all_clouds("/Users/seanngpack/Programming Stuff/Projects/scanner_files/17",
            CloudType::Type::RAW);

    delete controller;
    return 0;
}
