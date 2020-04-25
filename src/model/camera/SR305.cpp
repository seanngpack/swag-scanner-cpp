#include <SR305.h>
#include <librealsense2/rs.hpp>
#include <iostream>


camera::SR305::SR305() {

}

camera::ss_intrinsics camera::SR305::get_instrinsics() {
    int i = 0;
}

rs2::depth_frame camera::SR305::get_depth_frame() {
    int x = 0;
}

std::vector<float> camera::SR305::get_depth_vector() {
    int x = 0;
}

camera::SR305::~SR305() {
    std::cout << "calling SR305 destructor \n";
}

void camera::SR305::initialize_camera() {
    rs2::context ctx;
//    auto deviceList =
}
