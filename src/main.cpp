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
    const camera::ss_intrinsics *intrinsics = camera->get_instrinsics();

    model::Model *model = new model::Model();
    model->set_depth_frame(depth_frame);
    model->set_intrinsics(intrinsics);

//    pcl::PointCloud<pcl::Normal>::Ptr normals = model->estimate_normal_cloud();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = model->get_point_cloud();
    model->to_file(cloud, "test", CloudType::Type::RAW);


    delete camera;

    visual::Visualizer viewer;
    viewer.simpleVis(cloud);
//    viewer.normalsVis(cloud, normals);

    return 0;
}
