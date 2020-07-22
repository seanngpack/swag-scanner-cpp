#include <MoveController.h>

controller::MoveController::MoveController(std::shared_ptr<arduino::Arduino> arduino) :
        arduino(std::move(arduino)) {}

void controller::MoveController::run() {
    if (move_method == "to") {
        arduino->rotate_to(deg);
    }
    else if (move_method == "by") {
        arduino->rotate_by(deg);
    }
}

void controller::MoveController::set_deg(int degs) {
    this->deg = degs;
}

void controller::MoveController::set_move_method(const std::string &input) {
    this->move_method = input;
}
