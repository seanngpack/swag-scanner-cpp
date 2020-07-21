#include "ArduinoEventHandler.h"
#include "IFileHandler.h"
#include "json.hpp"

using json = nlohmann::json;

handler::ArduinoEventHandler::ArduinoEventHandler() :
        bluetooth_object(get_bluetooth_obj()),
        is_bt_connected(false),
        is_table_rotating(false),
        current_pos(file::IFileHandler::load_settings_json()["current_position"]) {
    set_handler(this, bluetooth_object);
    std::cout << "intialized current position is: " << current_pos <<std::endl;
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
    std::cout << "rotating..." << degs << std::endl;
    rotate(bluetooth_object, degs);
    current_pos += degs;
    current_pos %= 360;
    update_current_pos();
    table_cv.wait(ul, [this]() { return !is_table_rotating; });

}

void handler::ArduinoEventHandler::rotate_to(int pos) {
    int deg;
    if (pos < current_pos) {
        deg = 360 - abs(current_pos - pos);
        std::cout << "ned to rotate:" << deg << std::endl;
        rotate_table(deg);
    } else if (pos > current_pos) {
        deg = abs(current_pos - pos);
        std::cout << "ned to rotate:" << deg << std::endl;
        rotate_table(deg);
    }
    else {
        std::cout << "no need to rotate, already at set position" << std::endl;
    }
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

void handler::ArduinoEventHandler::update_current_pos() {
    json settings_json = file::IFileHandler::load_settings_json();
    settings_json["current_position"] = current_pos;
    file::IFileHandler::write_settings_json(settings_json);
}




