#include "Controller.h"

controller::Controller::Controller(camera::ICamera *camera,
                                   arduino::Arduino *arduino,
                                   model::Model *model,
                                   visual::Visualizer *viewer) :
        camera(camera), model(model), arduino(arduino), viewer(viewer) {}


void controller::Controller::scan(int degs) {
    if (360 % degs != 0) {
        throw std::invalid_argument("Invalid input, scanning input must be a factor of 360");
    }
    int num_rotations = 360 / degs;

    const camera::ss_intrinsics *intrin = camera->get_intrinsics();
    std::cout << "starting scanning..." << std::endl;
    for (int i = 0; i < num_rotations; i++) {
        std::string name = std::to_string(i*degs);
        const uint16_t *depth_frame = camera->get_depth_frame();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = model->create_point_cloud(depth_frame, intrin);
        model->to_file(cloud, name, CloudType::Type::RAW);
        arduino->rotate_table(degs);
    }


}

void controller::Controller::process_data() {
}

void controller::Controller::visualize_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    viewer->simpleVis(cloud);
}

controller::Controller::~Controller() {
    delete camera;
    delete arduino;
    delete model;
    delete viewer;
}




