#include "MoveController.h"
#include "Arduino.h"

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

void controller::MoveController::set_move_method(const MoveMethod &move_method) {
    this->move_method = move_method;
}
