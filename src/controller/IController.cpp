#include "IController.h"
#include <iostream>

void controller::IController::run() {
// do nothing
}

controller::IController::~IController() {
    {
        std::cout << "IController destructor" << std::endl;
    }
}
