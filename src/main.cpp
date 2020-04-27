#include <iostream>
#include <Model.h>
#include "SR305.h"
#include <chrono>

int main() {
    std::cout << "Hello, World!" << std::endl;
    camera::ICamera *camera = new camera::SR305();
    const uint16_t *depth_frame = camera->get_depth_frame();
    camera::ss_intrinsics *intrinsics = camera->get_instrinsics();

    model::Model *model = new model::Model();
    model->set_depth_frame(depth_frame);
    model->set_intrinsics(intrinsics);
    auto t1 = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = model->create_point_cloud();
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
    std::cout << "deprojection and cloud creation done in: ";
    std::cout << duration;
    std::cout << " milliseconds" << std::endl;
    std::cout << cloud->width << std::endl;
    std::cout << cloud->height << std::endl;
    std::cout << cloud->size() << std::endl;

    delete camera;
    return 0;
}
