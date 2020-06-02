#include <iostream>
#include <Model.h>
#include <SR305.h>
#include <CloudType.h>
#include "Visualizer.h"
#include <chrono>


int main() {
    std::cout << "Hello, World!" << std::endl;
    camera::ICamera *camera = new camera::SR305();
    const uint16_t *depth_frame = camera->get_depth_frame();
    const camera::ss_intrinsics *intrinsics = camera->get_intrinsics();

    model::Model *model = new model::Model();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = model->create_point_cloud(depth_frame, intrinsics);
//    model->to_file(cloud, "test", CloudType::Type::RAW);
//    pcl::PointCloud<pcl::Normal>::Ptr normals = model->estimate_normal_cloud(cloud);

    visual::Visualizer viewer;
    model->crop_cloud(cloud,
                      -.15, .15,
                      -100, .08,
                      -100, .5);
    viewer.simpleVis(cloud);
//    viewer.normalsVis(cloud, normals);
    delete camera;
    return 0;
}
