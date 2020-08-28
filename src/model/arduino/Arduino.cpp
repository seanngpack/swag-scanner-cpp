#include "Arduino.h"

#include "IFileHandler.h"
#include "../../../extern/json.hpp"
#include <thread>

using json = nlohmann::json;
using namespace std::literals::chrono_literals;

arduino::Arduino::Arduino() {
    current_pos = file::IFileHandler::load_settings_json()["current_position"];

    central_manager = std::make_unique<bluetooth::Central>();
    central_manager->start_bluetooth();
    arduino = central_manager->find_peripheral(DEVICE_NAME);
    service = arduino->find_service(UART_SERVICE_UUID);
    rotate_char = service->find_characteristic(ROTATE_TABLE_CHAR_UUID);
    table_pos_char = service->find_characteristic(TABLE_POSITION_CHAR_UUID);
    is_table_rot_char = service->find_characteristic(IS_TABLE_ROTATING_CHAR_UUID);
}


void arduino::Arduino::rotate_by(int deg) {
    rotate_char->write_with_response<int>(deg);

    if (deg < 0) {
        deg += 360;
    }
    current_pos += deg;
    current_pos %= 360;
    update_current_pos();
    std::this_thread::sleep_for(.5s);
}

void arduino::Arduino::rotate_to(int target) {
    if (current_pos < target) {
        int forward = target - current_pos;
        int back = 360 + current_pos - target;
        rotate_by(get_least(forward, back));
    } else if (current_pos > target) {
        int forward = 360 + target - current_pos;
        int back = current_pos - target;
        rotate_by(get_least(forward, back));
    }
}


void arduino::Arduino::update_current_pos() {
    json settings_json = file::IFileHandler::load_settings_json();
    settings_json["current_position"] = current_pos;
    file::IFileHandler::write_settings_json(settings_json);
}


int arduino::Arduino::get_least(int x, int y) {
    if (x <= y) {
        return x;
    }
    else {
        return -y;
    }
}


