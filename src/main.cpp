#include <iostream>
#include "Model.h"
#include "SR305.h"
#include "Visualizer.h"
#include <chrono>
#import "CoreFoundation/CoreFoundation.h"


int main() {
    std::cout << "Hello, World!" << std::endl;
    camera::ICamera *camera = new camera::SR305();
    const uint16_t *depth_frame = camera->get_depth_frame();
    const camera::ss_intrinsics *intrinsics = camera->get_instrinsics();

    model::Model *model = new model::Model();
    model->set_depth_frame(depth_frame);
    model->set_intrinsics(intrinsics);

    auto t1 = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = model->create_point_cloud();
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    std::cout << "deprojection and cloud creation done in: ";
    std::cout << duration;
    std::cout << " milliseconds" << std::endl;
    std::cout << cloud->width << std::endl;
    std::cout << cloud->height << std::endl;
    std::cout << cloud->size() << std::endl;

    pcl::PointCloud<pcl::Normal>::Ptr normals = model->estimate_normal_cloud();

    delete camera;
    visual::Visualizer viewer;
    viewer.normalsVis(cloud, normals);

    return 0;
}
