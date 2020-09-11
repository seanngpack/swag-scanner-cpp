#include "MoveController.h"
#include "Arduino.h"
#include "IFileHandler.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

controller::MoveController::MoveController(std::shared_ptr<arduino::Arduino> arduino) :
        arduino(std::move(arduino)) {}

void controller::MoveController::run() {
    if (move_method == MoveMethod::TO) {
        arduino->rotate_to(deg);
    } else if (move_method == MoveMethod::BY) {
        arduino->rotate_by(deg);
    }
}

void controller::MoveController::set_deg(int degs) {
    this->deg = degs;
}

void controller::MoveController::set_home() {
    json settings_json = file::IFileHandler::load_swag_scanner_info_json();
    settings_json["current_position"] = 0;
    file::IFileHandler::write_swag_scanner_info_json(settings_json);
}

void controller::MoveController::set_move_method(const MoveMethod &move_method) {
    this->move_method = move_method;
}
