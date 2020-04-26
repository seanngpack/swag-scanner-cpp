#include <iostream>
#include <Model.h>
#include "SR305.h"

int main() {
    std::cout << "Hello, World!" << std::endl;
    camera::ICamera *camera = new camera::SR305();
    const uint16_t *depth_frame = camera->get_depth_frame();
    camera::ss_intrinsics *intrinsics = camera->get_instrinsics();

    model::Model *model = new model::Model();
    model->set_depth_frame(depth_frame);
    model->set_intrinsics(intrinsics);
    model->create_point_cloud();

    delete camera;
    return 0;
}
