#include <iostream>
#include "SR305.h"

int main() {
    std::cout << "Hello, World!" << std::endl;
    camera::ICamera* camera = new camera::SR305();
    delete camera;
    return 0;
}
