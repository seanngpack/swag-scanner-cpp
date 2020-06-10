#include "Arduino.h"
#include "CoreBluetoothWrapper.h"
#include <thread>

using namespace std::literals::chrono_literals;

arduino::Arduino::Arduino() {
    bluetooth_object = get_bluetooth_obj();
    set_rotation_state_callback(this, bluetooth_object);
    start_bluetooth(bluetooth_object);
    wait_for_connection();
}

void arduino::Arduino::rotate_table(int deg) {
    std::cout << "sending rotation command" << std::endl;
    rotate(bluetooth_object, deg);
    while(isRotating) {
        // set the timer to how many times you want to check
        // for the stop condition
        std::this_thread::sleep_for(.1s);
    }
    std::cout << "finished rotation" << std::endl;
}

void arduino::Arduino::setIsRotating(bool in) {
    if (in) {
        std::cout << "rotating table" << std::endl;
        isRotating = true;
    }
    else {
        std::cout << "not rotating table" << std::endl;
        isRotating = false;
    }
}

void arduino::Arduino::wait_for_connection() {
    while(!isConnected) {
        std::cout << "waiting for bluetooth to finish..." << std::endl;
        std::this_thread::sleep_for(2s);
    }
}

void arduino::Arduino::setIsConnected(bool in) {
    isConnected = in;
}
