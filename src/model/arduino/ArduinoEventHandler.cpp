#include "ArduinoEventHandler.h"

handler::ArduinoEventHandler::ArduinoEventHandler() :
        bluetooth_object(get_bluetooth_obj()),
        is_bt_connected(false),
        is_table_rotating(false) {
    set_handler(this, bluetooth_object);
}

void handler::ArduinoEventHandler::connect_bluetooth() {
    std::unique_lock<std::mutex> ul(bt_mutex);
    start_bluetooth(bluetooth_object);
    bt_cv.wait(ul, [this]() { return is_bt_connected; });
    using namespace std::literals::chrono_literals;
    std::this_thread::sleep_for(1s); // needs a hard delay to prevent bluetooth overloading
    std::cout << "Finally connected to bluetooth, unblocking thread" << std::endl;
}

void handler::ArduinoEventHandler::rotate_table(int degs) {
    using namespace std::literals::chrono_literals;
    std::unique_lock<std::mutex> ul(table_mutex);
    rotate(bluetooth_object, degs);
    table_cv.wait(ul, [this]() { return !is_table_rotating; });
}

void handler::ArduinoEventHandler::set_is_bt_connected(bool is_connected) {
    is_bt_connected = is_connected;
}

void handler::ArduinoEventHandler::set_is_table_rotating(bool is_rotating) {
    is_table_rotating = is_rotating;
}

handler::ArduinoEventHandler::~ArduinoEventHandler() {
    bluetooth_object = nullptr;
}


