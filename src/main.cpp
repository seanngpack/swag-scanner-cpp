#include <iostream>
#include <Model.h>
#include <SR305.h>
#include "Controller.h"
#include "Visualizer.h"
#include <chrono>


int main() {
    camera::ICamera *camera = new camera::SR305();
    arduino::Arduino *arduino = new arduino::Arduino();
    model::Model *model = new model::Model();
    visual::Visualizer *viewer = new visual::Visualizer();

    controller::Controller *controller = new controller::Controller(camera,
            arduino, model, viewer);
    controller->scan(9);

//    pcl::PointCloud<pcl::Normal>::Ptr normals = model->estimate_normal_cloud(cloud);


//    model->crop_cloud(cloud,
//                      -.15, .15,
//                      -100, .08,
//                      -100, .5);

    delete controller;
    return 0;
}
