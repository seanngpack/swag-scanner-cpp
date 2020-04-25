#include <iostream>
#include "SR305.h"

int main() {
    std::cout << "Hello, World!" << std::endl;
    camera::ICamera* camera = new camera::SR305();
    const uint16_t* depth_image = camera->get_depth_frame();

    for (int dy = 0; dy < 480; dy++) {
        for (int dx = 0; dx < 640; dx++) {
            uint16_t depth_value = depth_image[dy * 640 + dx];
            std::cout << depth_value;
            std::cout << "\n";
        }
    }
    delete camera;
    return 0;
}
